"""Forward RGB-D camera on the G1 torso (Isaac Sim 6.1 `isaacsim.sensors.experimental.rtx`).

Read-only sensor: produces RGB, metric depth, intrinsics, camera pose and
timestamps. It is not connected to navigation, locomotion or the speed profile.

Frames follow the OpenCV convention in metadata (`T_world_optical`: +Z forward,
+X right, +Y down). The USD camera prim itself looks down -Z with +Y up.
"""
from collections import deque
from datetime import datetime
from pathlib import Path
import json,math,time
import numpy as np
from perception.rgbd_config import load_config,ROOT

USD_TO_OPTICAL=np.diag([1.,-1.,-1.,1.])


def author_camera(stage,config,robot_root='/World/G1'):
    """Define the camera prim under the torso rigid body. Returns (path, mount matrix)."""
    from pxr import Gf,UsdGeom,UsdPhysics
    bodies=[p for p in stage.Traverse() if p.GetName()==config['parent_link'] and str(p.GetPath()).startswith(robot_root+'/')
            and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies)!=1: raise ValueError('RGB-D camera needs exactly one '+config['parent_link']+' rigid body')
    eye=Gf.Vec3d(*config['torso_offset_m']); pitch=math.radians(config['pitch_down_deg'])
    # Link frame: +X forward, +Z up. Positive pitch looks down.
    direction=Gf.Vec3d(math.cos(pitch),0,-math.sin(pitch))
    camera=UsdGeom.Camera.Define(stage,bodies[0].GetPath().AppendChild(config['prim_name']))
    mount=Gf.Matrix4d().SetLookAt(eye,eye+direction,Gf.Vec3d(0,0,1)).GetInverse()
    camera.AddTransformOp().Set(mount)
    width,height=config['resolution']; aperture=20.955
    camera.CreateHorizontalApertureAttr(aperture)
    camera.CreateVerticalApertureAttr(aperture*height/width)  # square pixels
    camera.CreateFocalLengthAttr(aperture/(2*math.tan(math.radians(config['horizontal_fov_deg'])/2)))
    camera.CreateClippingRangeAttr(Gf.Vec2f(*config['clipping_range_m']))
    return str(camera.GetPath()),mount


def matrix(gf):
    """USD row-vector matrix -> standard column-vector 4x4 (p_parent = T @ p_child)."""
    return np.array([list(row) for row in gf],dtype=np.float64).T


def colorize_depth(depth_m,valid,lo,hi):
    """Near = red, far = blue, invalid = black. uint8 HxWx3."""
    t=np.clip((depth_m-lo)/(hi-lo),0,1); t=1-np.sqrt(t)  # sqrt spreads the near range
    rgb=np.stack([np.clip(1.5-np.abs(4*t-3),0,1),np.clip(1.5-np.abs(4*t-2),0,1),np.clip(1.5-np.abs(4*t-1),0,1)],axis=-1)
    rgb[~valid]=0
    return (rgb*255).astype(np.uint8)


class RgbdCamera:
    def __init__(self,runtime,config):
        self.runtime=runtime; self.config=config
        self.path=runtime.rgbd_camera_path; self.mount=runtime.rgbd_mount
        self.width,self.height=config['resolution']
        self.sensor=None; self.frame=None; self.frame_index=0; self.saved=0
        self.every=max(1,round(runtime.loco_cfg['control_hz']/config['capture_hz']))
        self.ticks=0; self.poses=deque(maxlen=8); self.read_ms=0.
        self.directory=ROOT/config['save_directory']/runtime.directory.name
        self.window=None; self.providers=None; self.label=None
        self.overlay=None  # optional callable(rgb) -> annotated rgb, used by the debug window only
        self.costmap_view=None  # optional callable() -> costmap picture or None, debug window only
        self.meters_per_unit=1.

    # ---- lifecycle -------------------------------------------------------
    def start(self):
        """Create the render product and annotators. Call again after a stage reset."""
        from pxr import UsdGeom
        from isaacsim.sensors.experimental.rtx import RtxCamera,CameraSensor
        self.meters_per_unit=float(UsdGeom.GetStageMetersPerUnit(self.runtime.stage))
        camera=RtxCamera(self.path,reset_xform_op_properties=False)
        self.sensor=CameraSensor(camera,resolution=(self.height,self.width),annotators=['rgb','distance_to_image_plane'])
        self.poses.clear(); self.frame=None
        if self.config.get('debug_window') and self.window is None: self._build_window()

    def stop(self):
        if self.sensor is not None:
            self.sensor._invalidate_sensor(); self.sensor=None

    def close(self):
        self.stop()
        if self.window is not None:
            self.window.visible=False; self.window.destroy(); self.window=None; self.providers=None

    # ---- geometry --------------------------------------------------------
    def intrinsics(self):
        from pxr import UsdGeom
        camera=UsdGeom.Camera(self.runtime.stage.GetPrimAtPath(self.path))
        focal=camera.GetFocalLengthAttr().Get()
        fx=self.width*focal/camera.GetHorizontalApertureAttr().Get()
        fy=self.height*focal/camera.GetVerticalApertureAttr().Get()
        cx,cy=self.width/2,self.height/2
        return {'fx':fx,'fy':fy,'cx':cx,'cy':cy,'width':self.width,'height':self.height,
                'K':[[fx,0,cx],[0,fy,cy],[0,0,1]],'model':'pinhole, no distortion',
                'horizontal_fov_deg':math.degrees(2*math.atan(self.width/(2*fx))),
                'vertical_fov_deg':math.degrees(2*math.atan(self.height/(2*fy)))}

    def world_pose(self):
        """Current T_world_camera (USD camera axes) from the physics-updated stage."""
        from pxr import UsdGeom
        prim=self.runtime.stage.GetPrimAtPath(self.path)
        return matrix(UsdGeom.XformCache().GetLocalToWorldTransform(prim))

    # ---- acquisition -----------------------------------------------------
    def update(self):
        """Call once per runtime tick, after physics. Returns a new frame or None."""
        if self.sensor is None: return None
        runtime=self.runtime
        self.poses.append({'step':runtime.steps,'sim_time':runtime.steps/runtime.loco_cfg['control_hz'],'T':self.world_pose(),
                           'base_position_m':list(runtime.last_state['base_position_m']),
                           'base_orientation_wxyz':list(runtime.last_state['base_orientation_wxyz'])})
        self.ticks+=1
        if self.ticks%self.every: return None
        frame=self.read()
        if frame is not None and self.providers is not None: self._show(frame)
        return frame

    def read(self):
        started=time.perf_counter()
        rgb,_=self.sensor.get_data('rgb'); depth,_=self.sensor.get_data('distance_to_image_plane')
        if rgb is None or depth is None: return None
        rgb=np.array(rgb.numpy() if hasattr(rgb,'numpy') else rgb,dtype=np.uint8).reshape(self.height,self.width,-1)[:,:,:3]
        raw=np.array(depth.numpy() if hasattr(depth,'numpy') else depth,dtype=np.float32).reshape(self.height,self.width)
        # The annotator reports distance along the optical axis in stage units.
        depth_m=raw*self.meters_per_unit
        lo,hi=self.config['valid_depth_range_m']
        valid=np.isfinite(depth_m)&(depth_m>=lo)&(depth_m<=hi)
        # The image available now was rendered before the latest physics steps.
        lag=min(self.config['pose_lag_ticks'],len(self.poses)-1); pose=self.poses[-1-lag]
        self.frame_index+=1; self.read_ms=(time.perf_counter()-started)*1000
        self.frame={'index':self.frame_index,'rgb':rgb,'depth_m':depth_m,'valid':valid,'intrinsics':self.intrinsics(),
                    'T_world_camera_usd':pose['T'],'T_world_optical':pose['T']@USD_TO_OPTICAL,
                    'sim_time':pose['sim_time'],'control_step':pose['step'],'wall_time':time.time(),
                    'read_step':self.runtime.steps,'pose_lag_ticks':lag,
                    'base_position_m':pose['base_position_m'],'base_orientation_wxyz':pose['base_orientation_wxyz']}
        return self.frame

    def recent_poses(self): return list(self.poses)

    # ---- debug -----------------------------------------------------------
    def debug_image(self,frame):
        lo,hi=self.config['valid_depth_range_m']
        return np.concatenate([frame['rgb'],colorize_depth(frame['depth_m'],frame['valid'],lo,hi)],axis=1)

    def metadata(self,frame):
        depth=frame['depth_m']; valid=frame['valid']
        return {'camera_prim':self.path,'frame_index':frame['index'],'sim_time_s':frame['sim_time'],'control_step':frame['control_step'],
            'wall_time_s':frame['wall_time'],'wall_time_iso':datetime.fromtimestamp(frame['wall_time']).isoformat(),
            'pose_lag_ticks':frame['pose_lag_ticks'],'resolution_wh':[self.width,self.height],'intrinsics':frame['intrinsics'],
            'T_parent_camera_usd':matrix(self.mount).tolist(),'parent_link':self.config['parent_link'],
            'T_world_camera_usd':frame['T_world_camera_usd'].tolist(),'T_world_optical':frame['T_world_optical'].tolist(),
            'frames':{'world':'metres, Z up','camera_usd':'-Z forward, +Y up, +X right','optical':'+Z forward, +X right, +Y down (OpenCV)'},
            'matrix_convention':'4x4, column vectors: p_world = T_world_x @ p_x',
            'depth':{'type':'distance_to_image_plane (Z along optical axis, not ray length)','unit':'metre','dtype':'float32',
                'stage_meters_per_unit':self.meters_per_unit,'valid_range_m':self.config['valid_depth_range_m'],
                'valid_fraction':float(valid.mean()),'min_m':float(depth[valid].min()) if valid.any() else None,
                'max_m':float(depth[valid].max()) if valid.any() else None,
                'png16':'uint16 millimetres, 0 = invalid/out of range','npy':'float32 metres, raw (inf = no hit)'},
            'robot_base_position_m':frame['base_position_m'],'robot_base_orientation_wxyz':frame['base_orientation_wxyz']}

    def save(self,frame=None,tag='frame'):
        """Write RGB PNG, depth (.npy metres + 16-bit mm PNG), colour depth, side-by-side debug and metadata."""
        from PIL import Image
        frame=frame or self.frame
        if frame is None: raise RuntimeError('No RGB-D frame available yet')
        self.directory.mkdir(parents=True,exist_ok=True)
        self.saved+=1; stem=self.directory/f'{self.saved:03d}_{tag}'
        lo,hi=self.config['valid_depth_range_m']
        Image.fromarray(frame['rgb']).save(str(stem)+'_rgb.png')
        np.save(str(stem)+'_depth_m.npy',frame['depth_m'])
        millimetres=np.where(frame['valid'],np.round(frame['depth_m']*1000),0).astype(np.uint16)
        Image.fromarray(millimetres).save(str(stem)+'_depth_mm16.png')
        Image.fromarray(self.debug_image(frame)).save(str(stem)+'_debug.png')
        Path(str(stem)+'_meta.json').write_text(json.dumps(self.metadata(frame),indent=2))
        return stem

    def _build_window(self):
        import omni.ui as ui
        self.window=ui.Window('G1 RGB-D Camera | DEBUG',width=2*self.width//2+24,height=self.height//2+90)
        self.window.position_x=16; self.window.position_y=440
        yolo=self.runtime.factory_cfg.get('yolo',{}).get('enabled',False)
        costmap=self.runtime.factory_cfg.get('local_costmap',{}).get('enabled',False)
        self.providers={name:ui.ByteImageProvider() for name in ['rgb','depth']+(['yolo'] if yolo else [])+(['costmap'] if costmap else [])}
        if yolo: self.window.width=self.width+(650 if costmap else 0)+24; self.window.height=self.height//2+(self.height+140 if costmap else self.height)+100
        with self.window.frame:
            with ui.VStack(spacing=4):
                with ui.HStack(height=self.height//2):
                    for name in ('rgb','depth'): ui.ImageWithProvider(self.providers[name],width=self.width//2,height=self.height//2)
                with ui.HStack(height=self.height+140 if costmap else (self.height if yolo else 0)):
                    # Full-resolution RGB with YOLO boxes, class names, confidences and distances.
                    if yolo: ui.ImageWithProvider(self.providers['yolo'],width=self.width,height=self.height)
                    # Local costmap: occupied / free / unknown, inflation and factory no-go areas.
                    if costmap: ui.ImageWithProvider(self.providers['costmap'],width=630,height=498)
                self.label=ui.Label('RGB | Depth (near red, far blue, black invalid)',height=20)
                ui.Button('Save RGB + Depth + metadata',height=24,clicked_fn=lambda:self.save(tag='manual'))

    def _show(self,frame):
        # Half-resolution previews keep the UI upload cheap; saved files stay full resolution.
        lo,hi=self.config['valid_depth_range_m']
        panes={'rgb':frame['rgb'][::2,::2],'depth':colorize_depth(frame['depth_m'],frame['valid'],lo,hi)[::2,::2]}
        if 'yolo' in self.providers: panes['yolo']=self.overlay(frame['rgb']) if self.overlay is not None else frame['rgb']
        if 'costmap' in self.providers and self.costmap_view is not None:
            picture=self.costmap_view()
            if picture is not None: panes['costmap']=picture
        for name,image in panes.items():
            provider=self.providers[name]
            size=[image.shape[1],image.shape[0]]
            rgba=np.ascontiguousarray(np.concatenate([image,np.full((*image.shape[:2],1),255,dtype=np.uint8)],axis=-1))
            if hasattr(provider,'set_data_array'): provider.set_data_array(rgba,size)
            else: provider.set_bytes_data(rgba.flatten().tolist(),size)
        centre=frame['depth_m'][self.height//2,self.width//2]
        self.label.text=(f'frame {frame["index"]} | t={frame["sim_time"]:.2f} s | centre depth {centre:.2f} m | '
                         f'valid {100*frame["valid"].mean():.0f}% | saved {self.saved}')

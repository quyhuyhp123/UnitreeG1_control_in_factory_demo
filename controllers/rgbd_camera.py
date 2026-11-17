"""Optional RGB-D acquisition adapter for installed Isaac Sim 6.1 CameraSensor.

No navigation or policy commands. Depth is optical-axis Z, not ray range.
"""
from pathlib import Path
import json
import math
import time
from datetime import datetime, timezone
import numpy as np

ROOT = Path(r'D:\unitree_g1')


class RobotRgbdCamera:
    def __init__(self, stage, output, config=None):
        from pxr import UsdGeom
        self.stage = stage
        self.cfg = config or json.loads((ROOT/'configs/rgbd_camera.json').read_text())
        w,h = self.cfg['resolution_wh']
        if not (isinstance(w,int) and isinstance(h,int) and 0<w<=1920 and 0<h<=1080):
            raise ValueError('Invalid RGB-D resolution_wh')
        if not (0<self.cfg['capture_hz']<=1000 and 0<self.cfg['horizontal_fov_deg']<170):
            raise ValueError('Invalid RGB-D capture frequency or FOV')
        near,far = self.cfg['clipping_m']
        if not 0<near<far or self.cfg['debug_depth_max_m']<=0:
            raise ValueError('Invalid RGB-D depth range')
        self.output = Path(output)/'rgbd'
        self.sensor = None
        self.window = None
        self.latest = None
        self.error = None
        self.last_poll = -float('inf')
        self.frames = 0
        self.yolo=None
        self.toggle_yolo=None
        from controllers.obstacle_localization import DepthLocalizer
        self.localizer=DepthLocalizer()
        self.obstacles=None
        from controllers.local_costmap import LocalCostmap
        self.costmap=LocalCostmap()
        if abs(UsdGeom.GetStageMetersPerUnit(stage)-1.0) > 1e-8:
            raise ValueError('RGB-D adapter requires a meter-based stage; refusing ambiguous depth units')
        if self.cfg['enabled']:
            self.enable()

    def enable(self):
        if self.sensor is not None:
            return
        from pxr import UsdGeom, Gf, Sdf
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension('isaacsim.sensors.experimental.rtx')
        enable_extension('omni.pip.compute')
        from isaacsim.sensors.experimental.rtx import CameraSensor
        import warp as wp
        parent = Sdf.Path(self.cfg['prim_path']).GetParentPath()
        if not self.stage.GetPrimAtPath(parent):
            raise ValueError(f'Camera mount prim missing: {parent}')
        camera = UsdGeom.Camera.Define(self.stage, self.cfg['prim_path'])
        eye = Gf.Vec3d(*self.cfg['mount_position_m'])
        pitch = math.radians(self.cfg['pitch_down_deg'])
        target = eye + Gf.Vec3d(math.cos(pitch), 0, -math.sin(pitch))
        self.mount = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0,0,1)).GetInverse()
        xf = UsdGeom.Xformable(camera)
        xf.ClearXformOpOrder()
        xf.AddTransformOp().Set(self.mount)
        camera.CreateHorizontalApertureAttr(20.955)
        camera.CreateFocalLengthAttr(20.955/(2*math.tan(math.radians(self.cfg['horizontal_fov_deg']/2))))
        camera.CreateClippingRangeAttr(Gf.Vec2f(*self.cfg['clipping_m']))
        self.camera = camera
        from pxr import UsdPhysics
        bounds_cache=UsdGeom.BBoxCache(0,['default','render','proxy'],useExtentsHint=True)
        self.costmap.static_bounds=[]
        for prim in (self.stage.Traverse() if self.cfg.get("processing_enabled",True) else []):
            if str(prim.GetPath()).startswith('/World/G1') or not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():continue
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):continue
            bounds=bounds_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if not bounds.IsEmpty() and bounds.GetMax()[2]>.06 and bounds.GetMin()[2]<2:
                self.costmap.static_bounds.append([list(bounds.GetMin()),list(bounds.GetMax())])
        w, h = self.cfg['resolution_wh']
        # New 6.1 API uses (height, width), unlike viewport API.
        self.sensor = CameraSensor(self.cfg['prim_path'], resolution=(h,w),
                                   annotators=['rgb','distance_to_image_plane'])
        self.rgb_buffer = wp.empty((h,w,3), dtype=wp.uint8, device='cpu')
        self.depth_buffer = wp.empty((h,w,1), dtype=wp.float32, device='cpu')
        self.error = None
        if self.cfg['debug'] and self.window is None:
            self._build_debug()
        if self.window is not None:
            self.window.visible = True
        if self.cfg.get('processing_enabled',True) and self.cfg['debug'] and self.costmap.window is None:self.costmap.debug()

    def disable(self):
        if self.sensor is not None:
            # Installed 6.1 SensorRuntime has no public close(). This adapter
            # owns its render product and uses the same teardown as __del__.
            self.sensor._invalidate_sensor()
            self.sensor = None
        if self.window is not None:
            self.status.text = 'RGB-D disabled | Enable sensor to resume acquisition'

    def _build_debug(self):
        import omni.ui as ui
        self.window = ui.Window('G1 Camera | RGB + optional YOLO', width=490, height=340)
        self.window.position_x=400
        self.window.position_y=64
        self.providers = [ui.ByteImageProvider()]
        with self.window.frame:
            with ui.VStack():
                self.status = ui.Label('Warming up RGB-D...', height=24)
                with ui.HStack(height=180):
                    for provider in self.providers:
                        ui.ImageWithProvider(provider)
                with ui.HStack(height=28):
                    ui.Button('Save RGB + depth + metadata', clicked_fn=self.save)
                    ui.Button('Enable sensor', clicked_fn=self.enable)
                    ui.Button('Disable sensor', clicked_fn=self.disable)
                    ui.Button('YOLO on/off', clicked_fn=lambda:self.toggle_yolo() if self.toggle_yolo else None)

    def poll(self, sim_time_s):
        if self.sensor is None:
            return None
        now = time.perf_counter()
        if now-self.last_poll < 1/self.cfg['capture_hz']:
            return None
        self.last_poll = now
        rgb, rgb_info = self.sensor.get_data('rgb', out=self.rgb_buffer)
        depth, depth_info = self.sensor.get_data('distance_to_image_plane', out=self.depth_buffer)
        if rgb is None or depth is None:
            return None
        from pxr import UsdGeom
        w,h = self.cfg['resolution_wh']
        f = self.camera.GetFocalLengthAttr().Get()
        ha = self.camera.GetHorizontalApertureAttr().Get()
        va = self.camera.GetVerticalApertureAttr().Get()
        T = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(self.camera.GetPrim())).T
        parent_T = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(self.camera.GetPrim().GetParent())).T
        optical = T @ np.diag([1.,-1.,-1.,1.])
        robot_prim=self.stage.GetPrimAtPath('/World/G1/pelvis')
        robot_T=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(robot_prim)).T
        from pxr import UsdPhysics
        cache=UsdGeom.BBoxCache(0,['default','render','proxy'],useExtentsHint=True)
        self_bounds=[]
        for prim in (self.stage.Traverse() if self.cfg.get("processing_enabled",True) else []):
            if str(prim.GetPath()).startswith('/World/G1/') and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bounds=cache.ComputeWorldBound(prim).ComputeAlignedRange()
                if not bounds.IsEmpty(): self_bounds.append([list(bounds.GetMin()),list(bounds.GetMax())])
        self.frames += 1
        metadata = {
            'prim_path': self.cfg['prim_path'], 'resolution_wh': [w,h],
            'depth_unit': 'meter', 'depth_kind': 'distance_to_image_plane (optical Z)',
            'invalid_depth': 'nonfinite or <= 0',
            'intrinsics_K': [[f/ha*w,0,(w-1)/2],[0,f/va*h,(h-1)/2],[0,0,1]],
            'T_world_usd_camera': T.tolist(), 'T_world_optical': optical.tolist(),
            'T_world_mount_parent': parent_T.tolist(),
            'T_world_robot': robot_T.tolist(),
            'robot_frame': '/World/G1/pelvis; +X forward +Y left +Z up',
            'robot_self_aabbs_world_m': self_bounds,
            'matrix_convention': 'column vectors; optical +X right +Y down +Z forward',
            'acquisition_sim_time_s': sim_time_s,
            'acquisition_utc': datetime.now(timezone.utc).isoformat(),
            'acquisition_unix_s': time.time(),
            'timestamp_scope': 'host acquisition after synchronous app.update; not hardware exposure timestamp',
            'rgb_annotator_info': rgb_info, 'depth_annotator_info': depth_info,
            'frame_counter': self.frames,
            'mount_position_m': self.cfg['mount_position_m'], 'pitch_down_deg': self.cfg['pitch_down_deg']}
        self.latest = (rgb.numpy().copy(), depth.numpy()[...,0].copy(), metadata)
        if self.cfg.get('processing_enabled',True) and self.localizer.cfg['enabled']:
            self.obstacles=self.localizer.geometry(self.latest[1],metadata)
            self.costmap.update(self.latest[1],metadata,self.localizer)
            self.output.mkdir(parents=True,exist_ok=True)
            with (self.output/'geometric_obstacles.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps({'frame_counter':self.frames,'timestamp':metadata['acquisition_utc'],'objects':self.obstacles})+'\n')
        if self.window is not None and self.window.visible:
            import cv2
            image=self.latest[0];state='YOLO off'
            if self.yolo is not None and self.yolo.enabled:
                annotated=self.yolo.annotated()
                state='YOLO: '+(self.yolo.error or ('ready' if self.yolo.ready else 'loading'))
                if annotated is not None:
                    image,result=annotated
                    state+=f" | frame {result['frame_counter']} | {result['inference_total_ms']:.0f} ms"
            preview=cv2.resize(image,(480,270))
            rgba=np.concatenate([preview,np.full((270,480,1),255,np.uint8)],axis=2)
            self.providers[0].set_bytes_data(rgba.ravel().tolist(),[480,270])
            self.status.text=f'RGB {w}x{h} | {state}'
        return self.latest

    def depth_color(self, depth):
        import cv2
        valid = np.isfinite(depth) & (depth > 0)
        normalized = np.zeros(depth.shape, dtype=np.uint8)
        normalized[valid] = (255*(1-np.clip(depth[valid]/self.cfg['debug_depth_max_m'],0,1))).astype(np.uint8)
        color = cv2.cvtColor(cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
        color[~valid] = 0
        return color

    def save(self, name=None):
        if self.latest is None:
            return None
        import cv2
        rgb, depth, metadata = self.latest
        directory = self.output/(name or datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        directory.mkdir(parents=True, exist_ok=False)
        if hasattr(self.costmap,'grid'):self.costmap.save(directory/'costmap')
        cv2.imwrite(str(directory/'rgb.png'), cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        np.save(directory/'depth_m.npy', depth)
        valid = np.isfinite(depth) & (depth > 0)
        mm = np.zeros(depth.shape, dtype=np.uint16)
        mm[valid] = np.clip(depth[valid]*1000,1,65535).astype(np.uint16)
        cv2.imwrite(str(directory/'depth_mm.png'), mm)
        color = self.depth_color(depth)
        cv2.imwrite(str(directory/'debug_rgb_depth.png'), cv2.cvtColor(np.concatenate([rgb,color],axis=1),cv2.COLOR_RGB2BGR))
        (directory/'metadata.json').write_text(json.dumps(metadata,indent=2,default=str),encoding='utf-8')
        return directory

    def close(self):
        if self.costmap.window is not None:self.costmap.window.destroy();self.costmap.window=None
        self.disable()
        if self.window is not None:
            self.window.destroy()
            self.window = None

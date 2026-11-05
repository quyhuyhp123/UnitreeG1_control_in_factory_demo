"""Verify camera edits survive physics/keyboard updates and robot reset."""
from pathlib import Path
import sys,json
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    from run_factory_g1 import preflight
    cfg=json.loads((ROOT/'configs/factory_g1.json').read_text()); preflight(cfg,False)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
        '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        from controllers.g1_locomotion import inverse_rotate_wxyz
        from pxr import Gf,UsdGeom
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        camera=runtime.context['camera']; viewport=runtime.context['viewport']
        def advance(seconds):
            for _ in range(round(seconds*50)): app.update(); runtime.tick()
        advance(1)
        # Exercise the alternate xform representation a viewport editor can use.
        camera.ClearXformOpOrder()
        camera.AddTranslateOp().Set(Gf.Vec3d(-1,-3,2.5))
        camera.AddRotateXYZOp().Set(Gf.Vec3f(65,0,0))
        def pose(): return UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim())
        edited=pose()
        before=runtime.last_state
        runtime.inject('W'); advance(3.5); runtime.inject('W','RELEASE'); advance(1.5)
        import numpy as np
        delta=float(inverse_rotate_wxyz(before['base_orientation_wxyz'],np.array(runtime.last_state['base_position_m'])-before['base_position_m'])[0])
        result={'input':'Carb logical keyboard; camera edited through USD, not OS mouse automation',
            'free_default':cfg['camera']['mode']=='free','camera_edit_survives_robot_motion':pose()==edited,
            'robot_moved_forward_m':delta,'robot_keyboard_physics_works':delta>.2 and runtime.fault is None}
        runtime.timeline.pause(); app.update(); reset=runtime.safe_reset(source='camera_regression')
        result['camera_edit_survives_reset']=reset and pose()==edited
        runtime.set_camera_mode('follow'); tracked=pose()
        result['follow_button_changes_view']=tracked!=edited
        runtime.set_camera_mode('free'); runtime.update_camera()
        result['free_button_preserves_current_view']=pose()==tracked
        result['status']='passed' if all(result[k] for k in ['free_default','camera_edit_survives_robot_motion',
            'robot_keyboard_physics_works','camera_edit_survives_reset','follow_button_changes_view','free_button_preserves_current_view']) else 'failed'
        result['run_directory']=str(runtime.directory)
        (ROOT/'logs/free_camera_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print('FREE_CAMERA_RESULT '+json.dumps(result),flush=True)
        if result['status']!='passed': raise RuntimeError('Free camera checks failed')
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()

"""Check head preview hides editor gizmos even when a scene object is selected."""
from pathlib import Path
import sys,json,time,argparse
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--hold-seconds',type=float,default=20); args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
        '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        import omni.usd
        from omni.kit.viewport.window import get_viewport_window_instances
        from controllers.factory_g1_runtime import FactoryG1Runtime
        cfg=json.loads((ROOT/'configs/factory_g1.json').read_text())
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        selected=next(p for p in runtime.stage.Traverse() if p.GetName().startswith('Spawn_Border_X_'))
        omni.usd.get_context().get_selection().set_selected_prim_paths([str(selected.GetPath())],False)
        for _ in range(60): app.update(); runtime.tick()
        main=next(w for w in get_viewport_window_instances() if w.viewport_api==runtime.context['viewport'])
        head=runtime.head_window._find_viewport_layer('Prim Transform','manipulator')
        main_item=main._find_viewport_layer('Prim Transform','manipulator')
        result={'selected_prim':str(selected.GetPath()),'head_transform_gizmo_hidden':head is not None and not head.visible,
            'main_transform_visibility':main_item.visible if main_item else None,
            'hidden_preview_layers':runtime.head_hidden_overlays,'run_directory':str(runtime.directory)}
        runtime.capture_head_preview(runtime.directory/'head_camera_clean.png')
        result['status']='passed' if result['head_transform_gizmo_hidden'] else 'failed'
        (ROOT/'logs/head_overlay_result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print('HEAD_OVERLAY_RESULT '+json.dumps(result),flush=True)
        until=time.perf_counter()+args.hold_seconds
        while app.is_running() and time.perf_counter()<until: app.update(); runtime.tick(); time.sleep(.02)
    except BaseException:
        import traceback
        print(traceback.format_exc(),flush=True); raise
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()

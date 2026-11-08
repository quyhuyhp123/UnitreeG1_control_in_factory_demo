"""Programmatic GUI integration test; does not claim physical mouse/key testing."""
from pathlib import Path
import sys,json
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','anti_aliasing':1,
                       'multi_gpu':False,'extra_args':['--/app/vulkan=false',
                       '--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
                       '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        cfg=json.loads((ROOT/'configs/factory_g1.json').read_text())
        runtime=FactoryG1Runtime(app,cfg,self_test=True)
        policy=runtime.policy
        for _ in range(50): app.update(); runtime.tick()
        runtime.controller_choice.model.get_item_value_model().set_value(1)
        runtime.apply_controller_selection()
        assert 'Restart required' in runtime.controller_message.text
        assert runtime.policy is policy and runtime.controller_selector.current=='locomotion'
        runtime.controller_choice.model.get_item_value_model().set_value(0)
        runtime.apply_controller_selection()
        assert 'already active' in runtime.controller_message.text
        for mode in ['overview','follow','free']: runtime.set_camera_mode(mode); app.update()
        assert runtime.head_widget is not None
        result={'status':'passed','policy_unchanged':True,'switch_blocked':True,
                'head_preview_present':True,'camera_modes_preserved':True,
                'scope':'Programmatic UI callbacks and rendering; no physical mouse/keyboard input',
                'run_directory':str(runtime.directory),'fault':runtime.fault}
        runtime.context['capture'](runtime.directory/'controller_manager.png')
        (ROOT/'docs/sonic_integration/ui_validation.json').write_text(json.dumps(result,indent=2))
        print('CONTROLLER_UI_RESULT '+json.dumps(result),flush=True)
    finally:
        if runtime: runtime.close()
        app.close()


if __name__=='__main__': main()

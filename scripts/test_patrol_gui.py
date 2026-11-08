"""GUI callback smoke test and pause-clock follow-up. Optional hold for visual QA."""
from pathlib import Path
import sys,json,time,argparse
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--hold-seconds',type=float,default=0); args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':False,'renderer':'RaytracedLighting','anti_aliasing':1,'multi_gpu':False,'extra_args':[
        '--/app/vulkan=false','--/exts/isaacsim.physics.newton/auto_switch_on_startup=false',
        '--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None; result={}
    try:
        from controllers.factory_g1_runtime import FactoryG1Runtime
        runtime=FactoryG1Runtime(app,json.loads((ROOT/'configs/factory_g1.json').read_text()),self_test=True)
        nav=runtime.navigation; panel=runtime.patrol_panel
        def steps(n):
            for _ in range(n): app.update(); runtime.tick()
        steps(50)
        runtime.timeline.pause(); app.update(); runtime.tick(); sim_before=nav.sim_time
        steps(10); result['simulation_clock_frozen']=nav.sim_time==sim_before
        nav.scheduler.config['clock_mode']='wall'; nav.before_tick(False); wall_before=nav.now
        steps(10); result['wall_clock_advances_while_paused']=nav.now>wall_before
        result['no_robot_steps_while_paused']=runtime.steps==round(sim_before*50)
        runtime.timeline.play(); app.update(); steps(20)
        nav.scheduler.config['clock_mode']='simulation'; nav.before_tick(True)
        panel.points['B'][0].set_value(0); panel.points['B'][1].set_value(-2.5)
        panel.apply_points(); result['gui_waypoints_valid']=nav.b[:2]==[0,-2.5]
        panel.start(); result['start_callback']=nav.scheduler.state=='WAITING_START'
        nav.pause(); result['pause_callback']=nav.scheduler.state=='PAUSED'
        nav.resume(); result['resume_callback']=nav.scheduler.state=='WAITING_START'
        nav.stop(); nav.set_mode('MANUAL'); panel.mode.model.get_item_value_model().set_value(0)
        nav.return_to_a(); steps(60)
        result['return_to_a_callback']=nav.scheduler.state=='STOPPED' and nav.scheduler.completed==0
        nav.set_mode('MANUAL')
        panel.top_view(); steps(5)
        result['top_view_active']=str(runtime.context['viewport'].camera_path)=='/World/Navigation/TopCamera'
        result['head_camera_present']=runtime.head_widget is not None
        for mode in ['follow','overview','free']: runtime.set_camera_mode(mode); steps(2)
        result['camera_modes_preserved']=True
        panel.top_view(); runtime.timeline.pause(); app.update()
        runtime.safe_reset('patrol_gui_top_view_reset'); runtime.timeline.play(); app.update(); steps(5)
        result['top_view_survives_reset']=str(runtime.context['viewport'].camera_path)=='/World/Navigation/TopCamera'
        nav.validate_points(); panel.top_view(); panel.draw(); steps(2)
        runtime.context['capture'](nav.directory/'patrol_gui_top_view.png')
        result['status']='passed' if all(result.values()) else 'failed'
        result['run_directory']=str(runtime.directory); result['patrol_directory']=str(nav.directory)
        result['scope']='Programmatic UI callbacks and actual GUI rendering; native clicks separately documented'
        (ROOT/'docs/navigation/gui_test_results.json').write_text(json.dumps(result,indent=2))
        print('PATROL_GUI_RESULT '+json.dumps(result),flush=True)
        until=time.perf_counter()+args.hold_seconds
        while app.is_running() and time.perf_counter()<until:
            app.update(); runtime.tick()
    finally:
        if runtime: runtime.close()
        app.close()


if __name__=='__main__': main()

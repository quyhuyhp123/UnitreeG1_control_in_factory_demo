"""Interactive G1 keyboard control, or explicit Carb-input integration tests."""
from pathlib import Path
import argparse,json,math,sys,time,traceback
PROJECT_ROOT=Path(r'D:\unitree_g1')
sys.path.insert(0,str(PROJECT_ROOT))
sys.path.insert(0,str(PROJECT_ROOT/'scripts'))

def yaw(q):
    w,x,y,z=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def integration_tests(runtime):
    import numpy as np
    report={'status':'running','input_source':'Carb InputProvider logical keyboard; not OS SendInput','cases':[],
            'policy_unchanged':True,'run_directory':str(runtime.directory)}
    result_path=PROJECT_ROOT/'logs/keyboard_test_results.json'
    def save():
        payload=json.dumps(report,indent=2)
        result_path.write_text(payload,encoding='utf-8')
        (runtime.directory/'test_results.json').write_text(payload,encoding='utf-8')
    def advance(seconds):
        rows=[]
        for _ in range(round(seconds*runtime.loco_cfg['control_hz'])):
            runtime.app.update()
            row=runtime.tick()
            if row: rows.append(row)
        return rows
    def release():
        for key in runtime.keys.mapping: runtime.inject(key,'RELEASE')
    def reset_case():
        release()
        runtime.synthetic_focus=True
        runtime.binding.refresh_focus()
        runtime.timeline.pause()
        runtime.app.update()
        assert runtime.safe_reset(source='between_independent_test_cases')
        runtime.timeline.play()
        runtime.app.update()
        runtime.keys.set_running(True)
        advance(.5)
    def add(number,name,command_pass,motion_pass=None,**details):
        case={'test':number,'name':name,'command_pass':bool(command_pass),'motion_pass':None if motion_pass is None else bool(motion_pass),**details}
        report['cases'].append(case)
        save()
        print('KEYBOARD_TEST '+json.dumps(case),flush=True)
    def motion_case(number,name,keys,axis,sign,seconds=3.5):
        reset_case()
        before=runtime.policy.feedback.read_feedback()
        for key in keys: runtime.inject(key,'PRESS')
        rows=advance(seconds)
        held_end=runtime.policy.feedback.read_feedback()
        release()
        advance(1.0)
        if not rows:
            add(number,name,False,False,error=runtime.fault or 'No control frames')
            return
        commands=np.array([r['applied_command'] for r in rows])
        command_pass=float(sign*commands[-1,axis])>.05
        displacement=np.array(held_end['base_position_m'])-np.array(before['base_position_m'])
        angles=np.unwrap([yaw(r['base_orientation_wxyz']) for r in rows]+[yaw(held_end['base_orientation_wxyz'])])
        turn=float(angles[-1]-angles[0])
        delta=float(displacement[axis]) if axis in [0,1] else turn
        motion_pass=sign*delta>(.2 if axis in [0,1] else .12) and runtime.fault is None
        if number in [10,11]:
            command_pass=commands[-1,0]>.1 and sign*commands[-1,2]>.05
            motion_pass=displacement[0]>.2 and sign*turn>.12 and runtime.fault is None
        add(number,name,command_pass,motion_pass,displacement_world_m=displacement.tolist(),yaw_change_rad=turn,
            final_held_command=commands[-1].tolist(),reset_after_case_only=True,policy_fault=runtime.fault)
    save()
    for item in [(1,'W forward',['W'],0,1),(2,'Arrow Up forward',['UP'],0,1),
                 (3,'S backward',['S'],0,-1),(4,'Arrow Down backward',['DOWN'],0,-1),
                 (5,'A pure yaw left',['A'],2,1),(6,'Arrow Left pure yaw',['LEFT'],2,1),
                 (7,'D pure yaw right',['D'],2,-1),(8,'Arrow Right pure yaw',['RIGHT'],2,-1),
                 (9,'Q lateral left',['Q'],1,1),(9,'E lateral right',['E'],1,-1),
                 (10,'W+A forward and yaw left',['W','A'],2,1),(11,'Up+Right forward and yaw right',['UP','RIGHT'],2,-1)]:
        motion_case(*item)
    reset_case()
    runtime.inject('W'); runtime.inject('S')
    rows=advance(1)
    add(12,'W+S cancels vx',all(r['desired_command'][0]==0 and r['applied_command'][0]==0 for r in rows),None)
    release()
    runtime.inject('LEFT'); runtime.inject('RIGHT'); rows=advance(.2)
    lr=all(r['applied_command'][2]==0 for r in rows)
    release(); runtime.inject('Q'); runtime.inject('E'); rows=advance(.2)
    add('12-extra','Left+Right and Q+E cancel',lr and all(r['applied_command'][1]==0 for r in rows))
    reset_case(); runtime.inject('W'); advance(2)
    release(); rows=advance(1.5)
    commands=[runtime.key_cfg['max_forward_velocity']]+[r['applied_command'][0] for r in rows]
    smooth=all(abs(b-a)<=runtime.keys.decel[0]/runtime.loco_cfg['control_hz']+1e-6 for a,b in zip(commands,commands[1:]))
    add(13,'Release decelerates command to zero',smooth and abs(commands[-1])<1e-9,None,final_command=runtime.keys.command)
    reset_case(); runtime.inject('W'); advance(2)
    runtime.inject('SPACE'); rows=advance(.02)
    zero=rows and rows[0]['applied_command']==[0,0,0]
    runtime.inject('SPACE','RELEASE'); runtime.inject('W','REPEAT'); more=advance(.4)
    add(14,'SPACE immediate zero; stale hold cannot restart',zero and all(r['applied_command']==[0,0,0] for r in more))
    release()
    reset_case(); runtime.inject('W'); advance(2)
    release(); runtime.timeline.pause(); runtime.app.update()
    count=len(runtime.reset_events)
    runtime.inject('R'); runtime.tick()
    record=runtime.reset_events[-1] if len(runtime.reset_events)>count else {}
    reset_ok=(len(runtime.reset_events)==count+1 and np.linalg.norm(record.get('position_after_reset',[99,99,99])[:2])<.03
              and record.get('history_terms_after_reset')==0 and record.get('policy_steps_after_reset')==0 and record.get('safe_before_reset'))
    add(15,'R resets safely while paused, including history',reset_ok,None,reset_record=record)
    runtime.inject('R','RELEASE'); runtime.timeline.play(); runtime.app.update()
    reset_case(); runtime.inject('W'); advance(2)
    runtime.synthetic_focus=False; runtime.binding.refresh_focus(); rows=advance(.1)
    zero=all(r['applied_command']==[0,0,0] for r in rows)
    runtime.synthetic_focus=True; runtime.binding.refresh_focus(); runtime.inject('W','REPEAT'); rows=advance(.4)
    no_stale=all(r['applied_command']==[0,0,0] for r in rows)
    runtime.inject('W','RELEASE'); runtime.inject('W'); rows=advance(.3)
    add(16,'Focus loss clears old command; fresh press required',zero and no_stale and rows[-1]['applied_command'][0]>0,
        None,focus_source='Injected focus gate; native app focus requires manual validation')
    release()
    reset_case(); runtime.inject('W'); advance(1)
    runtime.timeline.pause(); runtime.app.update()
    old_position=runtime.last_state['base_position_m'][:]
    runtime.inject('S'); advance(.5)
    paused_zero=runtime.keys.command==[0,0,0]
    no_motion=np.linalg.norm(np.array(runtime.last_state['base_position_m'])-old_position)<1e-9
    runtime.timeline.play(); runtime.app.update(); runtime.inject('W','REPEAT'); rows=advance(.3)
    no_resume=all(r['applied_command']==[0,0,0] for r in rows)
    release(); runtime.inject('W'); rows=advance(.3)
    add('17','Pause/Resume rejects stale/paused keys',paused_zero and no_motion and no_resume and rows[-1]['applied_command'][0]>0)
    release()
    subscription=runtime.binding.subscription
    duplicate=runtime.binding.start()
    before=runtime.keys.event_count
    runtime.inject('W'); runtime.inject('W','RELEASE')
    add('18','Idempotent subscription',duplicate is False and runtime.binding.subscription==subscription and runtime.keys.event_count-before==2)
    import carb.input
    observed=[]
    spy=runtime.input.subscribe_to_input_events(lambda e:(observed.append(e.event.input.name) or True),device=runtime.logical_keyboard,order=-1)
    runtime.inject('UP'); runtime.inject('UP','RELEASE')
    blocked=not observed
    runtime.provider.buffer_keyboard_key_event(runtime.logical_keyboard,carb.input.KeyboardEventType.KEY_PRESS,carb.input.KeyboardInput.X,0)
    runtime.provider.update_keyboard(runtime.logical_keyboard); runtime.input.distribute_buffered_events()
    runtime.input.unsubscribe_to_input_events(spy)
    add('19','Arrow capture blocks later camera listeners; unrelated keys pass',blocked and observed==['X'],later_events=observed)
    release()
    report['all_command_tests_passed']=all(c['command_pass'] for c in report['cases'])
    report['policy_motion_limitations']=[{'test':c['test'],'name':c['name']} for c in report['cases'] if c['motion_pass'] is False]
    report['all_motion_tests_passed']=not report['policy_motion_limitations']
    report['status']='passed_commands_with_policy_motion_limitations' if report['all_command_tests_passed'] else 'failed_keyboard_logic'
    if report['all_command_tests_passed'] and report['all_motion_tests_passed']: report['status']='passed_all'
    save()
    print('KEYBOARD_RESULT '+json.dumps({'status':report['status'],'policy_motion_limitations':report['policy_motion_limitations'],'run_directory':str(runtime.directory)}),flush=True)
    return report

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--duration',type=float,default=0,help='Interactive auto-close seconds, 0=until window closed')
    args,_=parser.parse_known_args()
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':args.self_test and not args.gui,'width':1100,'height':760,'renderer':'RaytracedLighting','multi_gpu':False,
        'extra_args':['--/exts/isaacsim.physics.newton/auto_switch_on_startup=false','--/exts/isaacsim.core.simulation_manager/default_engine=physx']})
    runtime=None
    try:
        from controllers.g1_keyboard_runtime import G1KeyboardRuntime
        runtime=G1KeyboardRuntime(app,logical_test_keyboard=args.self_test)
        print('KEYBOARD_READY '+str(runtime.directory),flush=True)
        if args.self_test:
            result=integration_tests(runtime)
            if not result['all_command_tests_passed']: raise RuntimeError('Keyboard integration checks failed')
        else:
            started=time.perf_counter()
            next_frame=started
            while app.is_running():
                app.update()
                runtime.tick()
                if args.duration and time.perf_counter()-started>=args.duration: break
                next_frame+=1/runtime.loco_cfg['control_hz']
                delay=next_frame-time.perf_counter()
                if delay>0: time.sleep(min(delay,.02))
                elif delay<-.1: next_frame=time.perf_counter()
    except BaseException:
        error=traceback.format_exc()
        (PROJECT_ROOT/'logs/keyboard_error.txt').write_text(error,encoding='utf-8')
        print(error,flush=True)
        raise
    finally:
        if runtime: runtime.close()
        app.close()

if __name__=='__main__': main()

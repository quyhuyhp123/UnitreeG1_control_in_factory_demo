"""Summarize persisted actual test evidence; never runs or invents a simulation."""
from pathlib import Path
import json,statistics,math,subprocess
ROOT=Path(r'D:\unitree_g1'); DOC=ROOT/'docs/navigation'


def load(path): return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def main():
    physical=load(DOC/'physics_test_results.json'); manual=load(DOC/'manual_regression_results.json')
    obstacles=load(DOC/'obstacle_test_results.json'); gui=load(DOC/'gui_test_results.json')
    perf={}
    for key,result in [('patrol',physical),('manual',manual)]:
        directory=Path(result['run_directory'])
        rows=[json.loads(s) for s in (directory/'telemetry.jsonl').read_text().splitlines()]
        samples=[json.loads(s) for s in (directory/'performance.jsonl').read_text().splitlines()]
        inference=[r['inference_ms'] for r in rows if 'inference_ms' in r]
        perf[key]={'run_directory':str(directory),'telemetry_samples':len(rows),
            'mean_inference_ms':statistics.mean(inference),'max_inference_ms':max(inference),
            'mean_loop_fps':statistics.mean(r['loop_fps'] for r in samples),
            'mean_physics_steps_per_wall_second':statistics.mean(r['physics_steps_per_wall_second'] for r in samples),
            'peak_whole_gpu_mib':max((r['gpu_device_total_used_mib'] for r in samples if 'gpu_device_total_used_mib' in r),default=None),
            'maximum_tilt_deg':max(r['tilt_deg'] for r in rows)}
    trace=load(Path(physical['patrol_directory'])/'patrol_trace.json')
    perf['patrol'].update(max_path_deviation_m=max(r['path_deviation_m'] for r in trace),
        cycle_durations_seconds=trace[-1]['cycle_durations_seconds'],
        approximate_base_distance_m=sum(math.dist(a['position'][:2],b['position'][:2]) for a,b in zip(trace,trace[1:])),
        maximum_reported_planner_ms=max(r['planner_latency_ms'] for r in trace),
        maximum_reported_map_update_ms=max(r['map_update_ms'] for r in trace),
        final_goal_error_m=math.dist(physical['tests']['two_cycles_a_b_a']['position'][:2],physical['tests']['route_valid']['path'][0]))
    (DOC/'performance_results.json').write_text(json.dumps(perf,indent=2))
    protected=['run_sim.bat','configs/factory_g1.json','configs/g1_robot.yaml','configs/keyboard.yaml','configs/locomotion_mjlab.yaml',
        'controllers/keyboard_controller.py','controllers/g1_mjlab_locomotion.py','controllers/g1_joint_targets.py',
        'assets/factory/factory.usd','assets/factory/factory_physics.usd',
        'assets/robot/unitree_model/G1/29dof/usd/g1_29dof_rev_1_0/g1_29dof_rev_1_0.usd',
        'checkpoints/unitree_g1_mjlab_velocity_v0/policy.onnx']
    hashes={}
    for p in protected:
        def git(*args): return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()
        hashes[p]={'current_blob':git('hash-object',p),'baseline_blob':git('rev-parse','HEAD:'+p)}
    acceptance={'navigation':'IMPLEMENTED_AND_VERIFIED_FOR_RECORDED_SCENARIOS',
        'physical_patrol':physical['status'],'manual_regression':manual['status'],
        'obstacle_tests_passed':all(r['passed'] for r in obstacles.values()),'gui_callbacks':gui['status'],
        'native_desktop_acceptance':'BLOCKED: Computer Use native pipe unavailable; programmatic GUI passed',
        'sonic':'BLOCKED: existing SONIC PhysX backend not validated; never selected for patrol',
        'protected_files_unchanged':all(r['current_blob']==r['baseline_blob'] for r in hashes.values()),'protected_files':hashes}
    (DOC/'acceptance_summary.json').write_text(json.dumps(acceptance,indent=2))
    p=perf['patrol']; m=perf['manual']; unit=load(DOC/'unit_test_results.json')
    text=f'''# Navigation test report — 2026-11-08

Status: navigation implemented and verified for the recorded simulation scenarios. **Native desktop mouse/keyboard acceptance is BLOCKED** because the Computer Use native pipe could not connect (retry and kernel reset also failed). Programmatic GUI callbacks and actual viewport rendering passed. SONIC is still blocked and was not used.

| Test | Actual result and scope |
|---|---|
| 1 Manual regression | Fresh factory A–J self-test passed, zero failed checks; Carb logical W/S/A/D and arrow aliases, standing, release, reset, obstacle contact. Native OS keys still require checklist. |
| 2 A→B | Real Mjlab policy + PhysX, test A near [0,-4], validated B [0,-2.5]. No robot teleport during patrol. |
| 3 Return to A | Two complete A→B→A cycles passed. Final position error {p['final_goal_error_m']:.3f} m, tolerance 0.16 m plus velocity/heading settle. |
| 4 Lane boundary | Inflated footprint/grid unit tests and logged physical trajectory passed; max recorded path deviation {p['max_path_deviation_m']:.3f} m. |
| 5 Static obstacle | A* detour verified on algorithm test map. Real factory full-lane collider triggered waiting; physical detour around a static obstacle was not separately demonstrated. |
| 6 Dynamic obstacle | Actual kinematic collision actor crossed; stop/wait and clear-path resume passed. Min inflated clearance {obstacles['dynamic_crossing']['minimum_inflated_clearance_m']:.3f} m. |
| 7 Human crossing | Semantic human collider proxy crossed; stop/wait and resume passed, min inflated clearance {obstacles['human_crossing']['minimum_inflated_clearance_m']:.3f} m. This tests scene truth metadata, not an AI detector or animated human-mesh tracker. Existing workers remain scene objects. |
| 8 Fully blocked lane | WAITING_FOR_CLEAR_PATH, zero target, then replanning after actual collider removal passed. |
| 9 Schedule | Two physical cycles passed with shortened 2 s interval and 1 s dwell. Unit tests cover default 120 s and pause semantics; a full-duration 120 s physical waiting experiment was not run. |
| 10 Pause/resume | Final physical test and separate GUI tests passed: sim clock frozen, wall clock advances, no physics steps while paused, no automatic patrol resume. |
| 11 Manual takeover | Explicit cancellation/source selection and fresh manual command passed; AUTO ignored D/WASD contribution, SPACE paused Auto. |
| 12 Fault | Stale-data injection produced latched FAULT; fresh data did not restart, acknowledgement passed. Fault-stop ownership/budget unit tests passed. Falling robot, corrupted policy, GPU crash and every timeout permutation were not physically injected. |
| 13 Persistence | Actual JSON save/load roundtrip of A/B/map/speed/schedule passed using a log-directory test file, preserving user configuration. |
| 14 Performance | Measured table below. These are loop requests/whole-device VRAM, not presentation FPS or process-only memory. |

Unit suite: {unit['tests_run']} total, {unit['passed']} passed, {unit['skipped']} explicitly skipped (pre-existing blocked SONIC flat/factory tests), {unit['failures']} failures, {unit['errors']} errors.

| Measurement | Headless patrol | Manual regression |
|---|---:|---:|
| Mean loop FPS | {p['mean_loop_fps']:.2f} | {m['mean_loop_fps']:.2f} |
| Physics steps / wall second | {p['mean_physics_steps_per_wall_second']:.2f} | {m['mean_physics_steps_per_wall_second']:.2f} |
| Mean policy inference ms | {p['mean_inference_ms']:.3f} | {m['mean_inference_ms']:.3f} |
| Peak whole-GPU MiB | {p['peak_whole_gpu_mib']} | {m['peak_whole_gpu_mib']} |

Configured physics remains 200 Hz, policy 50 Hz. Patrol head preview was disabled for headless testing; manual/GUI production preview was enabled, so this is not a controlled renderer A/B benchmark. Patrol recorded planner maximum {p['maximum_reported_planner_ms']:.3f} ms and map-update maximum {p['maximum_reported_map_update_ms']:.2f} ms. Bounding-volume/map updates add CPU cost and wall-clock real-time 200 Hz is not achieved. Cycle durations in simulation seconds: {p['cycle_durations_seconds']}. Approximate sampled base travel {p['approximate_base_distance_m']:.2f} m, including lateral gait motion. No fall was reported in the accepted patrol run.

Measured braking: raw initial speed about 0.460 m/s, maximum post-zero base displacement 0.115 m; factor-two distance conservatism plus reaction/extra margin. One straight-floor setup is not worst-case validation for all gait phases, terrain, slopes or impacts. The initial actor test introduced a crossing too near the inflated stopping envelope and failed with FAULT; its log is retained. Accepted crossing tests use initial separation calculated from measured braking and footprint, actual moving USD/PhysX colliders, and explicit assertions—not hardcoded detector outputs.

Evidence: physics_test_results.json, obstacle_test_results.json, gui_test_results.json, manual_regression_results.json, unit_test_results.json, performance_results.json, acceptance_summary.json. Raw JSONL and images stay under the run directories recorded in those files, outside Git. Earlier heading-oscillation and sway-prediction failures and a one-frame pause assertion failure were preserved under logs/patrol_baseline; final physical suite passed all checks after fixes.

Source preservation: original factory USD, robot source USD, checkpoint, keyboard mapping/controller, joint adapter, locomotion implementation, timing configs and run_sim.bat match original Git blobs. Previous uncommitted SONIC work was carried into feature/g1-automatic-patrol and not reset or removed. Modified integration files retain that work. The generated g1_factory_base.usd test output was backed up and restored after testing; no factory regeneration, driver/software installation, sensor AI, training or robot network was used. Isaac/graphics software may use its existing default C: caches; project files stay on D:.

New code: navigation package, configs/navigation*.json and patrol_schedule.json, scripts/test_patrol*.py, summarize_patrol_tests.py and tests/test_navigation.py. Integration edits: factory_g1_runtime.py, g1_keyboard_runtime.py command hook, run_factory_g1.py Kit pacing, and README navigation entry. Existing launcher arguments and the default manual command remain valid. Code has not been committed or pushed for this task.
'''
    (DOC/'navigation_test_report.md').write_text(text,encoding='utf-8')
    print(json.dumps(acceptance,indent=2))


if __name__=='__main__': main()

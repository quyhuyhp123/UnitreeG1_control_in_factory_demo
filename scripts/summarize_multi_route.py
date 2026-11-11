"""Audit recorded PhysX traversal independently of configured completion history."""
import hashlib
import json
import re
import statistics
import sys
from pathlib import Path

ROOT = Path(r'D:\unitree_g1')
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from check_route_geometry import build_grid


def main():
    folder = ROOT / 'docs/navigation'
    physical = json.loads((folder / 'multi_route_physics_results.json').read_text())
    routes = json.loads((ROOT / 'configs/patrol_routes.json').read_text())
    if physical['status'] == 'running':
        raise SystemExit('Physical run is still running; no acceptance summary generated')
    trace = json.loads((Path(physical['patrol_directory']) / 'multi_route_trace.json').read_text())
    grid = build_grid()
    checks = {'physical_two_cycles': physical['status'] == 'passed' and physical['completed_cycles'] == 2}
    traversal = {}
    for cycle, rid in enumerate(('route_1_blue', 'route_2_brown')):
        rows = [r for r in trace if r['completed_cycles'] == cycle and r.get('current_segment')]
        edges = []
        for row in rows:
            if not edges or edges[-1] != row['current_segment']:
                edges.append(row['current_segment'])
        expected = routes['routes'][rid]['mandatory_segments']
        unsafe = [r['sim_time'] for r in rows if not grid.map.safe(r['position'][:2])]
        checks[rid + '_ordered_trace'] = edges == expected
        checks[rid + '_sampled_footprint_safe'] = not unsafe
        traversal[rid] = {'recorded_edges': edges, 'samples': len(rows), 'unsafe_sample_times': unsafe,
                          'mean_loop_fps': statistics.mean(r['loop_fps'] for r in rows),
                          'in_place_turn_samples': sum(r['state']=='TURNING' and abs(r['command'][2])>.1 and sum(v*v for v in r['command'][:2])<.0001 for r in rows),
                          'max_path_deviation_m': max((r['path_deviation_m'] for r in rows), default=None)}
    events = physical['transitions']
    completed_at = next(e['time'] for e in events if e['state']=='CYCLE_COMPLETE')
    restart_at = next(e['time'] for e in events if e['state']=='VALIDATING_ROUTE' and e['time']>completed_at)
    checks['test_repeat_interval_2s'] = abs(restart_at-completed_at-2.)<.021
    performance = [json.loads(line) for line in (Path(physical['run_directory'])/'performance.jsonl').read_text().splitlines()]
    physical['metrics']['mean_physics_steps_per_wall_second'] = statistics.mean(r['physics_steps_per_wall_second'] for r in performance)
    for name in ('ui', 'reset'):
        report = json.loads((folder / f'multi_route_{name}_results.json').read_text())
        checks[name] = report.get('status') == 'passed'
    legacy = json.loads((folder / 'multi_route_v1_regression.json').read_text())
    checks['v1_regression'] = legacy.get('status') == 'passed' or legacy.get('passed') is True
    protected = json.loads((ROOT / 'docs/factory_v2_integration_summary.json').read_text())
    hashes = {}
    for relative, before in protected['protected_files'].items():
        raw = (ROOT / relative).read_bytes()
        # sha256_worktree stores raw bytes; the flag describes the old Git
        # content comparison only, not the recorded worktree digest.
        actual = hashlib.sha256(raw).hexdigest()
        hashes[relative] = actual
        checks['unchanged:' + relative] = actual == before['sha256_worktree']
    source = ROOT / 'assets/factory/factory_layout_v2/factory_layout_v2.usd'
    checks['source_factory_unchanged'] = hashlib.sha256(source.read_bytes()).hexdigest() == routes['source_factory_sha256']
    errors = {}
    for filename in ('multi_route_physics_final.log', 'multi_route_ui.log', 'multi_route_reset.log', 'multi_route_v1_regression.log'):
        lines = (ROOT / 'logs' / filename).read_text(errors='replace').splitlines()
        errors[filename] = [line for line in lines if re.search(r'\[(?:Error|Fatal)\]|Traceback \(most recent call last\)', line)]
        checks['clean_log:' + filename] = not errors[filename]
    result = {'status': 'passed' if all(checks.values()) else 'failed', 'checks': checks,
              'trace_audit': traversal, 'protected_sha256': hashes, 'error_lines': errors,
              'metrics': physical.get('metrics'),
              'repeat_interval_measured_sim_seconds': restart_at-completed_at,
              'scope': 'Sampled SIM ground-truth footprint audit, not continuous swept-mesh proof; GUI callbacks/logical input, not native OS clicks.'}
    (folder / 'multi_route_acceptance.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()

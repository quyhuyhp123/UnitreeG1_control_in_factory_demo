"""Verify recorded physical route traversal; leave the historical speed audit unchanged."""
from pathlib import Path
import sys,json,math,statistics,re
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from check_route_geometry import build_grid

def main():
    folder=ROOT/'docs/navigation'; raw=json.loads((folder/'corner_fix_alternate_080_results.json').read_text())
    if raw['status']=='running': raise SystemExit('Simulation is still running')
    trace=json.loads((Path(raw['patrol_directory'])/'multi_route_trace.json').read_text())
    routes=json.loads((ROOT/'configs/patrol_routes.json').read_text(encoding='utf-8-sig')); grid=build_grid()
    checks={'completed_two_routes':raw['status']=='passed' and raw['completed_cycles']==2 and raw['state']=='STOPPED' and not raw['fault']}
    edges={}
    for cycle,rid in enumerate(('route_1_blue','route_2_brown')):
        rows=[r for r in trace if r['completed_cycles']==cycle and r['current_segment']]; ids=[]
        for row in rows:
            if not ids or ids[-1]!=row['current_segment']: ids.append(row['current_segment'])
        edges[rid]=ids; checks[rid+'_order']=ids==routes['routes'][rid]['mandatory_segments']
        checks[rid+'_sampled_footprint']=all(grid.map.safe(r['position'][:2]) for r in rows)
    replay=json.loads((folder/'corner_stall_fix_results.json').read_text()); checks['exact_stall_replay']=replay['status']=='passed' and replay['completed_cycles']==0
    bottom=json.loads((folder/'bottom_corner_fix_results.json').read_text()); checks['bottom_corner_replay']=bottom['status']=='passed' and bottom['completed_cycles']==0
    error_lines=[line for line in (ROOT/'logs/corner_fix_alternate_080.log').read_text(errors='replace').splitlines() if re.search(r'\[(?:Error|Fatal)\]|Traceback \(most recent call last\)',line)]
    checks['clean_final_log']=not error_lines
    moving=[math.hypot(*r['actual_velocity'][:2]) for r in trace if math.hypot(*r['command'][:2])>.1]
    result={'status':'passed' if all(checks.values()) else 'failed','scope':'One real PhysX Route1 -> Route2 run, selected cap 0.8 m/s with adaptive reductions; sampled footprint verification, not certification at 1.0 m/s',
            'checks':checks,'ordered_edges':edges,'samples':len(trace),'simulation_seconds':raw['simulation_seconds'],
            'mean_actual_speed_while_commanded_m_s':statistics.mean(moving) if moving else None,
            'peak_sampled_actual_speed_m_s':max(math.hypot(*r['actual_velocity'][:2]) for r in trace),
            'terminal_recovery_samples':sum(r.get('terminal_recovery_step',False) for r in trace),
            'metrics':raw['metrics'],'error_lines':error_lines,'run_directory':raw['run_directory'],'patrol_directory':raw['patrol_directory']}
    (folder/'corner_stall_fix_verification.json').write_text(json.dumps(result,indent=2)); print(json.dumps(result,indent=2))
    if result['status']!='passed': raise SystemExit(1)

if __name__=='__main__': main()

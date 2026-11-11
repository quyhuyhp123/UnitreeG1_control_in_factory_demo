"""Independent edge-order/footprint audit for a completed speed candidate."""
from pathlib import Path
import json,sys,argparse,math,statistics
ROOT=Path(r'D:\unitree_g1'); sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from check_route_geometry import build_grid

def audit(speed):
    path=ROOT/f'docs/navigation/factory_speed_{speed:.2f}_results.json'
    report=json.loads(path.read_text()); grid=build_grid()
    trace=json.loads((Path(report['patrol_directory'])/'multi_route_trace.json').read_text())
    routes=json.loads((ROOT/'configs/patrol_routes.json').read_text())
    checks={'completed_two_cycles':report['status']=='passed' and report['completed_cycles']==2,
            'actually_commanded_candidate':report['metrics']['forward_samples_above_90pct']>=10}
    for cycle,rid in enumerate(('route_1_blue','route_2_brown')):
        rows=[r for r in trace if r['completed_cycles']==cycle and r['current_segment']]
        edges=[]
        for row in rows:
            if not edges or edges[-1]!=row['current_segment']: edges.append(row['current_segment'])
        checks[rid+'_edge_order']=edges==routes['routes'][rid]['mandatory_segments']
        checks[rid+'_safe_samples']=all(grid.map.safe(r['position'][:2]) for r in rows)
    result={'candidate_speed_m_s':speed,'status':'passed' if all(checks.values()) else 'failed','checks':checks,
            'sample_count':len(trace),'metrics':report['metrics'],'simulation_seconds':report['simulation_seconds'],
            'mean_moving_speed_m_s':statistics.mean(math.hypot(*r['actual_velocity'][:2]) for r in trace if math.hypot(*r['command'][:2])>.1),
            'raw_report':str(path),'scope':'Two complete routes, sampled footprint audit, one run per candidate; not proof for every disturbance.'}
    (ROOT/f'docs/navigation/factory_speed_{speed:.2f}_audit.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('--speed',type=float,required=True); a=p.parse_args()
    if audit(a.speed)['status']!='passed': raise SystemExit(1)

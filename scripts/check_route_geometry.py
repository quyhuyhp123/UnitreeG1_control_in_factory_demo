import json,sys
from pathlib import Path
ROOT=Path(r'D:\unitree_g1'); sys.path.insert(0,str(ROOT))
from navigation.route_manager import RouteManager
from navigation.lane_map import LaneMap
from navigation.occupancy_grid import OccupancyGrid

def build_grid():
 m=json.loads((ROOT/'configs/navigation_map_v2.json').read_text()); data=json.loads((ROOT/'assets/factory/factory_layout_v2/navigation_allowed_areas.json').read_text()); obs=[{'id':o['id'],'kind':'human' if o['semantic_class']=='person' else 'static','bounds':[o['bounds'][0][:2],o['bounds'][1][:2]]} for o in data['static_obstacles'] if o['bounds'][0][2]<1.9 and o['bounds'][1][2]>.06]
 return OccupancyGrid(LaneMap(m,obs,[[-10,-15],[10,15]]))
if __name__=='__main__':
 grid=build_grid(); manager=RouteManager(json.loads((ROOT/'configs/factory_v2.json').read_text())); report={}
 for rid in manager.data['routes']:
  try:
   p=manager.validate(grid,rid); report[rid]={'status':'VALID_GEOMETRY','length_m':p['length_m'],'ordered_nodes':p['waypoints'],'segments':[{'id':e['id'],'path':e['path']} for e in p['edges']]}
  except Exception as ex: report[rid]={'status':'INVALID_ROUTE_SEGMENT','error':str(ex)}
 (ROOT/'docs/navigation/multi_route_geometry.json').write_text(json.dumps(report,indent=2)); print(json.dumps({rid:{k:v for k,v in row.items() if k not in ['segments','ordered_nodes']} for rid,row in report.items()}))
 if any(row['status']!='VALID_GEOMETRY' for row in report.values()): raise SystemExit(1)

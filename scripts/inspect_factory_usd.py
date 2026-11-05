from pathlib import Path
import json
from isaacsim import SimulationApp
app=SimulationApp({'headless':True,'width':640,'height':480})
from pxr import Usd, UsdGeom
p=Path(r'D:\unitree_g1')
stage=Usd.Stage.Open(str(p/'assets/factory/factory.usd'))
rows=[]
for prim in stage.Traverse():
    rows.append({'path':str(prim.GetPath()),'type':prim.GetTypeName(),'custom':{a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.IsCustom()},'name':prim.GetDisplayName()})
report={'default':str(stage.GetDefaultPrim().GetPath()),'meters':UsdGeom.GetStageMetersPerUnit(stage),'up':str(UsdGeom.GetStageUpAxis(stage)),'prims':rows}
(p/'logs/factory_usd_inventory.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({**report,'prims':rows[:18]},indent=2))
app.close()

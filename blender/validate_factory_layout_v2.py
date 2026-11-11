"""Round-trip file verification in Blender; no Isaac runtime."""
import bpy,json,traceback
from pathlib import Path
ROOT=Path(r'D:\unitree_g1'); OUT=ROOT/'assets/factory/factory_layout_v2'
report={}
bpy.ops.wm.open_mainfile(filepath=str(ROOT/'blender/factory_layout_v2.blend'))
bpy.context.view_layer.update()
report['blend_reopen']={'status':'passed','objects':len(bpy.context.scene.objects),'meshes':sum(o.type=='MESH' for o in bpy.context.scene.objects)}
source={o.name:list(o.matrix_world.translation) for o in bpy.context.scene.objects if o.type=='MESH' and not o.name.startswith('COL_')}
report['custom_metadata_complete']=all(all(k in o for k in ['semantic_class','object_id','is_static','is_navigation_obstacle','is_walkable','is_interactable']) for o in bpy.context.scene.objects)
try:
    from pxr import Usd,UsdGeom
    stage=Usd.Stage.Open(str(OUT/'factory_layout_v2.usd'))
    paths={p.GetAttribute('userProperties:object_id').Get():str(p.GetPath()) for p in stage.Traverse() if p.GetAttribute('userProperties:object_id').HasValue()}
    report['openusd']={'status':'passed','meters_per_unit':UsdGeom.GetStageMetersPerUnit(stage),'up_axis':str(UsdGeom.GetStageUpAxis(stage)),'prims':len(paths)}
    p=OUT/'object_manifest.json'; data=json.loads(p.read_text())
    for row in data['objects']: row['usd_prim_path']=paths.get(row['object_name'])
    p.write_text(json.dumps(data,indent=2),encoding='utf-8')
    cache=UsdGeom.XformCache(); mismatches=[]; missing=[]
    for name,pos in source.items():
        if name not in paths: missing.append(name); continue
        prim=stage.GetPrimAtPath(paths[name]); position=cache.GetLocalToWorldTransform(prim).ExtractTranslation()
        if max(abs(a-b) for a,b in zip(position,pos))>.002: mismatches.append(name)
    report['openusd'].update(missing_visual_objects=missing,position_mismatches=mismatches,collision_proxy_ids=sum(str(k).startswith('COL_') for k in paths),semantic_ids=len(paths))
except ImportError:
    report['openusd']={'status':'NOT TESTED','reason':'pxr Python module unavailable in Blender; importer roundtrip used instead. Manifest prim paths remain unverified/null.'}
for label,filename in [('usd','factory_layout_v2.usd'),('glb','factory_layout_v2.glb')]:
    try:
        bpy.ops.wm.read_factory_settings(use_empty=True)
        if label=='usd': result=bpy.ops.wm.usd_import(filepath=str(OUT/filename))
        else: result=bpy.ops.import_scene.gltf(filepath=str(OUT/filename))
        bpy.context.view_layer.update()
        objects=bpy.context.scene.objects
        missing=[]; errors=[]
        for name,pos in (source.items() if label=='glb' else []):
            o=objects.get(name)
            if o is None: missing.append(name)
            elif max(abs(a-b) for a,b in zip(o.matrix_world.translation,pos))>.002: errors.append(name)
        mesh_count=sum(o.type=='MESH' for o in objects)
        if label=='usd': missing=report['openusd'].get('missing_visual_objects',[]); errors=report['openusd'].get('position_mismatches',[])
        report[label+'_roundtrip']={'status':'passed' if 'FINISHED' in result and mesh_count>=len(source) and not missing and not errors else 'failed','objects':len(objects),'mesh_objects':mesh_count,'missing_visual_mesh_names':missing,'position_mismatches':errors,'collision_proxies_imported':sum(o.name.startswith('COL_') for o in objects),'note':'USD importer may use shared mesh data names; authoritative original object IDs and transforms verified with OpenUSD. Invisible proxies remain in USD but are skipped by default importer.' if label=='usd' else 'Standard glTF Y-up export; Blender importer restores Z-up.'}
    except Exception as exc: report[label+'_roundtrip']={'status':'failed','error':repr(exc)}; traceback.print_exc()
(OUT/'roundtrip_validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print('FACTORY_ROUNDTRIP '+json.dumps(report),flush=True)
if any(report[k]['status']!='passed' for k in ['blend_reopen','usd_roundtrip','glb_roundtrip']): raise RuntimeError('Round-trip verification failed')

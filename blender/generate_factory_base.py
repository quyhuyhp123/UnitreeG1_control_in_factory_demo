r"""Rebuild the factory in a NEW Blender scene; no existing scene is deleted.
Run: blender.exe --background --python D:\unitree_g1\blender\generate_factory.py
Outputs stay under PROJECT_ROOT. Existing outputs are backed up before saving.
"""
from pathlib import Path
import bpy
import json
import math
import shutil
from datetime import datetime
from mathutils import Vector

PROJECT_ROOT = Path(r'D:\unitree_g1')
OUT = PROJECT_ROOT / 'assets' / 'factory'
BLEND = PROJECT_ROOT / 'blender' / 'factory.blend'
OUT.mkdir(parents=True, exist_ok=True)
STAMP = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

def backup(path):
    if path.exists():
        target = PROJECT_ROOT / 'logs' / 'factory_backups' / STAMP / path.name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)

scene = bpy.data.scenes.new('Factory_10m_' + STAMP)
bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0
scene.unit_settings.length_unit = 'METERS'
collections = {}
for name in ['Architecture', 'Machines', 'Worktables', 'Storage', 'Floor_Markings', 'Markers', 'Lighting', 'Presentation']:
    col = bpy.data.collections.new(name)
    scene.collection.children.link(col)
    collections[name] = col

def material(name, color, metallic=0.0):
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    bsdf = next(n for n in mat.node_tree.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Base Color'].default_value = (*color, 1)
    bsdf.inputs['Roughness'].default_value = .65
    bsdf.inputs['Metallic'].default_value = metallic
    mat.diffuse_color = (*color, 1)
    return mat

mats = {
    'floor': material('Concrete_Slate', (.19, .24, .28)),
    'wall': material('Wall_LightGrey', (.63, .70, .73)),
    'machine': material('Machine_Teal', (.025, .31, .34), .25),
    'metal': material('Powdercoat_Navy', (.045, .09, .15), .35),
    'wood': material('Worktop_Warm', (.53, .32, .13)),
    'yellow': material('Safety_Amber', (1, .65, .035)),
    'white': material('Wayfinding_White', (.8, .88, .9)),
    'screen': material('Display_Blue', (.025, .10, .17)),
    'crate': material('Storage_Grey', (.35, .43, .48)),
}

def link(obj, collection, kind, collision=False):
    collections[collection].objects.link(obj)
    obj['asset_type'] = kind
    obj['collision_candidate'] = collision
    return obj

def box(name, size, location, mat, col, kind, parent=None):
    # Vertex dimensions bake scale directly into the mesh; origin at its centre.
    x,y,z = [v/2 for v in size]
    mesh = bpy.data.meshes.new(name + '_Mesh')
    mesh.from_pydata([(-x,-y,-z),(-x,-y,z),(-x,y,-z),(-x,y,z),(x,-y,-z),(x,-y,z),(x,y,-z),(x,y,z)], [],
                     [(0,4,6,2),(1,3,7,5),(0,1,5,4),(2,6,7,3),(0,2,3,1),(4,5,7,6)])
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    link(obj,col,kind,kind not in ['marking','display'])
    obj.location = location
    if parent:
        obj.parent = parent
        obj.matrix_parent_inverse = parent.matrix_world.inverted()
    obj.data.materials.append(mats[mat])
    return obj

def group(name, col, kind):
    obj = bpy.data.objects.new(name,None)
    link(obj,col,kind)
    obj.empty_display_size=.2
    return obj

box('Floor',(10,10,.2),(0,0,-.1),'floor','Architecture','floor')
# Interior clear dimensions 10 x 10 m. Walls sit outside floor perimeter.
box('Wall_01',(10,.2,3),(0,5.1,1.5),'wall','Architecture','wall')
box('Wall_02',(.2,10.4,3),(-5.1,0,1.5),'wall','Architecture','wall')
box('Wall_03',(.2,10.4,3),(5.1,0,1.5),'wall','Architecture','wall')
# Fourth wall has a 2 m wide, full-height doorway centred on X=0.
box('Wall_04_Left',(4,.2,3),(-3,-5.1,1.5),'wall','Architecture','wall')
box('Wall_04_Right',(4,.2,3),(3,-5.1,1.5),'wall','Architecture','wall')

for i,x in enumerate([-3.15,3.15],1):
    body=box(f'Machine_{i:02}',(1.5,1,1.8),(x,2.8,.9),'machine','Machines','machine')
    box(f'Machine_{i:02}_Display',(.55,.025,.42),(x,2.2875,1.30),'screen','Machines','display')
    box(f'Machine_{i:02}_ServicePanel',(1.1,.025,.65),(x,2.2875,.55),'metal','Machines','display')

for i,x in enumerate([-3.15,3.15],1):
    root=group(f'Table_{i:02}','Worktables','table')
    box(f'Table_{i:02}_Top',(1.6,.85,.10),(x,0,.75),'wood','Worktables','table_part',root)
    for j,(dx,dy) in enumerate([(-.65,-.28),(-.65,.28),(.65,-.28),(.65,.28)],1):
        box(f'Table_{i:02}_Leg_{j:02}',(.09,.09,.70),(x+dx,dy,.35),'metal','Worktables','table_part',root)

for i,x in enumerate([-3.15,3.15],1):
    root=group(f'Shelf_{i:02}','Storage','shelf')
    for j,(dx,dy) in enumerate([(-.85,-.35),(-.85,.35),(.85,-.35),(.85,.35)],1):
        box(f'Shelf_{i:02}_Post_{j:02}',(.08,.08,2.2),(x+dx,-2.8+dy,1.1),'metal','Storage','shelf_part',root)
    for j,z in enumerate([.15,.85,1.55,2.15],1):
        box(f'Shelf_{i:02}_Deck_{j:02}',(1.62,.78,.06),(x,-2.8,z),'wood','Storage','shelf_part',root)
    for j,dx in enumerate([-.45,.35],1):
        box(f'Shelf_{i:02}_Bin_{j:02}',(.55,.48,.38),(x+dx,-2.8,.37),'crate','Storage','storage_bin',root)

# Continuous clear aisle is x in [-1.1,1.1]; entry clear width is 2 m.
for i,x in enumerate([-1.15,1.15],1):
    box(f'Aisle_Boundary_{i:02}',(.05,9.6,.002),(x,0,.001),'yellow','Floor_Markings','marking')
for i,y in enumerate([-4.4,-3.6,-2.8,-2,2,2.8,3.6,4.4],1):
    box(f'Aisle_Dash_{i:02}',(.05,.35,.002),(0,y,.001),'white','Floor_Markings','marking')
# Spawn is an EMPTY, never a solid/collider. Border lies outside 2x2 clear zone.
spawn=group('Robot_Spawn','Markers','spawn_marker')
spawn.location=(0,0,0)
spawn.empty_display_size=1
spawn['clear_zone_m']=[2.0,2.0]
for i,x in enumerate([-1.035,1.035],1):
    box(f'Spawn_Border_X_{i:02}',(.025,2.045,.002),(x,0,.001),'yellow','Floor_Markings','marking')
for i,y in enumerate([-1.035,1.035],1):
    box(f'Spawn_Border_Y_{i:02}',(2.045,.025,.002),(0,y,.001),'yellow','Floor_Markings','marking')

def light(name,loc,energy,size):
    data=bpy.data.lights.new(name,'AREA')
    data.energy=energy
    data.shape='DISK'
    data.size=size
    obj=bpy.data.objects.new(name,data)
    link(obj,'Lighting','light')
    obj.location=loc
light('Light_Key',(0,-1,8),2200,8)
light('Light_Fill',(-3,3,6),1000,5)
world=bpy.data.worlds.new('Factory_World')
scene.world=world
world.use_nodes=True
next(n for n in world.node_tree.nodes if n.type=='BACKGROUND').inputs['Color'].default_value=(.32,.40,.5,1)
next(n for n in world.node_tree.nodes if n.type=='BACKGROUND').inputs['Strength'].default_value=.5
cam_data=bpy.data.cameras.new('Overview_Camera')
cam=bpy.data.objects.new('Overview_Camera',cam_data)
link(cam,'Presentation','camera')
cam.location=(11,-15,23)
cam.rotation_euler=(Vector((0,0,.3))-cam.location).to_track_quat('-Z','Y').to_euler()
cam_data.type='ORTHO'
cam_data.ortho_scale=16.8
scene.camera=cam
scene.render.resolution_x=1400
scene.render.resolution_y=1400
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.render.filepath=str(OUT/'factory_overview.png')
scene.render.engine='BLENDER_EEVEE'  # read from the connected Blender 5.2 session
scene['project_root']=str(PROJECT_ROOT)
scene['aisle_width_m']=2.2
scene['entrance_width_m']=2.0
bpy.context.view_layer.update()

def bounds(obj):
    points=[]
    members=[obj] if obj.type=='MESH' else list(obj.children_recursive)
    for member in members:
        if member.type=='MESH':
            points.extend(member.matrix_world @ Vector(c) for c in member.bound_box)
    if not points:
        return None
    return ([min(p[i] for p in points) for i in range(3)], [max(p[i] for p in points) for i in range(3)])

errors=[]
meshes=[o for o in scene.objects if o.type=='MESH']
for obj in meshes:
    if not obj.data.vertices or not obj.data.polygons: errors.append('empty mesh: '+obj.name)
    if any(abs(s-1)>1e-6 for s in obj.scale): errors.append('unapplied scale: '+obj.name)
    if not all(math.isfinite(v) for row in obj.matrix_world for v in row): errors.append('nonfinite transform: '+obj.name)
    lo,hi=bounds(obj)
    if obj.name!='Floor' and lo[0]<1 and hi[0]>-1 and lo[1]<1 and hi[1]>-1 and hi[2]>0:
        errors.append('spawn intrusion: '+obj.name)
# Strict AABB positive-volume overlap: touching faces are permitted.
overlaps=[]
for i,a in enumerate(meshes):
    aa,ab=bounds(a)
    for b in meshes[i+1:]:
        ba,bb=bounds(b)
        if all(min(ab[k],bb[k])-max(aa[k],ba[k])>1e-5 for k in range(3)):
            overlaps.append([a.name,b.name])
if overlaps: errors.append('positive-volume object overlaps')
report={'errors':errors,'overlaps':overlaps,'mesh_count':len(meshes),'polygons':sum(len(o.data.polygons) for o in meshes),
        'units':'meters','unit_scale':1.0,'up_axis':'Z','floor_top_z':0,'aisle_width_m':2.2,'entrance_width_m':2,
        'spawn_clear_size_m':[2,2],'physics_note':'No collision schemas authored by Blender exporter; assign static colliders in Isaac Sim before robot simulation.'}
backup(OUT/'validation_report.json')
(OUT/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if errors: raise RuntimeError(json.dumps(report))
manifest=[]
for obj in sorted(scene.objects,key=lambda o:o.name):
    bb=bounds(obj)
    manifest.append({'name':obj.name,'type':obj.get('asset_type',obj.type),'blender_type':obj.type,
        'location_m':list(obj.matrix_world.translation),'dimensions_m':[bb[1][i]-bb[0][i] for i in range(3)] if bb else [0,0,0],
        'bounds_world_m':bb,'scale':list(obj.scale),'parent':obj.parent.name if obj.parent else None,
        'collection':[c.name for c in obj.users_collection],'collision_candidate':bool(obj.get('collision_candidate',False))})
backup(OUT/'object_manifest.json')
(OUT/'object_manifest.json').write_text(json.dumps({'project_root':str(PROJECT_ROOT),'units':'meters','up_axis':'Z','objects':manifest},indent=2),encoding='utf-8')

backup(BLEND)
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND))
backup(OUT/'factory.usd')
try:
    result=bpy.ops.wm.usd_export(filepath=str(OUT/'factory.usd'),export_animation=False,export_lights=True,
        export_cameras=False,export_materials=True,generate_preview_surface=True,
        root_prim_path='/Factory',meters_per_unit=1.0,convert_orientation=False)
    if 'FINISHED' not in result: raise RuntimeError(str(result))
    export_status='USD exported successfully'
except Exception as exc:
    backup(OUT/'factory.glb')
    bpy.ops.export_scene.gltf(filepath=str(OUT/'factory.glb'),export_format='GLB')
    export_status=f'USD failed: {exc}; GLB fallback exported. Convert GLB via Isaac Sim Asset Converter to USD, then validate units and static colliders.'
backup(OUT/'export_status.txt')
(OUT/'export_status.txt').write_text(export_status,encoding='utf-8')
backup(OUT/'factory_overview.png')
bpy.ops.render.render(write_still=True)
print(json.dumps({'blend':str(BLEND),'export':export_status,'validation':report},indent=2))

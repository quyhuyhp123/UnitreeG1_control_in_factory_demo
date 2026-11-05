"""Industrial hall revision; all generated files remain in D:/unitree_g1."""
from pathlib import Path
BASE=Path(r'D:\unitree_g1\blender\generate_factory_base.py')
exec(compile(BASE.read_text(encoding='utf-8').split('def bounds(obj):')[0],str(BASE),'exec'))
scene.name='Industrial_Hall_Green_'+STAMP
for name in ['Steel_Frame','Roof','Utilities','Workers','Props','Signage']:
    col=bpy.data.collections.new(name)
    scene.collection.children.link(col)
    collections[name]=col
def color(key,rgb,roughness=.5):
    shader=next(n for n in mats[key].node_tree.nodes if n.type=='BSDF_PRINCIPLED')
    shader.inputs['Base Color'].default_value=(*rgb,1)
    shader.inputs['Roughness'].default_value=roughness
    mats[key].diffuse_color=(*rgb,1)
color('floor',(.025,.25,.085),.34)
color('wall',(.36,.42,.44))
color('white',(1,.65,.035))
for key,rgb in [('steel',(.12,.17,.20)),('panel',(.49,.55,.57)),('skin',(.58,.33,.19)),('uniform',(.035,.13,.29)),('orange',(.95,.25,.025)),('black',(.02,.025,.03)),('red',(.65,.025,.018)),('lamp',(.9,.96,1))]:
    mats[key]=material('Industrial_'+key,rgb,.2 if key in ['steel','panel'] else 0)
shader=next(n for n in mats['lamp'].node_tree.nodes if n.type=='BSDF_PRINCIPLED')
shader.inputs['Emission Color'].default_value=(.8,.9,1,1)
shader.inputs['Emission Strength'].default_value=3
def beam(name,a,b,w,d,mat,col,kind='structure'):
    a,b=Vector(a),Vector(b)
    obj=box(name,(w,d,(b-a).length),(a+b)/2,mat,col,kind)
    obj.rotation_euler=(b-a).to_track_quat('Z','Y').to_euler()
    return obj
def cyl(name,r,h,loc,mat,col,kind='prop'):
    bpy.ops.mesh.primitive_cylinder_add(vertices=16,radius=r,depth=h,location=loc)
    obj=bpy.context.object
    obj.name=name
    for c in list(obj.users_collection): c.objects.unlink(obj)
    link(obj,col,kind)
    obj.data.materials.append(mats[mat])
    return obj
def label(name,body,loc,size,mat):
    data=bpy.data.curves.new(name,'FONT')
    data.body=body
    data.align_x='CENTER'
    data.size=size
    obj=bpy.data.objects.new(name,data)
    link(obj,'Signage','sign')
    obj.location=loc
    obj.rotation_euler=(math.pi/2,0,0)
    data.materials.append(mats[mat])
    return obj
# 3 m lower wall panels retained; upper metal infill / frames form a taller hall.
for side,x in [('L',-4.82),('R',4.82)]:
    for j,y in enumerate([-4.65,-2.3,0,2.3,4.65],1):
        box(f'Column_{side}_{j}',(.18,.24,4.6),(x,y,2.3),'steel','Steel_Frame','structure')
        box(f'Baseplate_{side}_{j}',(.32,.38,.12),(x,y,.06),'yellow','Steel_Frame','structure')
    for j,z in enumerate([.6,1.5,2.4,3.15,4.25],1):
        box(f'Girt_{side}_{j}',(.09,9.8,.08),(x*1.022,0,z),'steel','Steel_Frame','structure')
    box(f'Upper_Wall_{side}',(.12,10,1.6),(x*1.045,0,3.8),'panel','Steel_Frame','structure')
    for j,y in enumerate([-3.5,-1.15,1.15,3.5],1):
        box(f'Clerestory_{side}_{j}',(.035,1.7,.65),(x*1.029,y,3.65),'screen','Steel_Frame','structure')
box('Upper_Wall_Back',(10,.15,1.6),(0,5.075,3.8),'panel','Steel_Frame','structure')
for j,y in enumerate([-4.65,-2.3,0,2.3,4.65],1):
    beam(f'Rafter_L_{j}',(-4.82,y,4.6),(0,y,5.15),.16,.15,'steel','Steel_Frame')
    beam(f'Rafter_R_{j}',(0,y,5.15),(4.82,y,4.6),.16,.15,'steel','Steel_Frame')
    box(f'Tie_Beam_{j}',(9.64,.1,.1),(0,y,4.48),'steel','Steel_Frame','structure')
for j,x in enumerate([-4,-2,0,2,4],1):
    box(f'Purlin_{j}',(.08,10,.1),(x,0,5.15-abs(x)*.55/4.82),'steel','Steel_Frame','structure')
for side,s in [('L',-1),('R',1)]:
    obj=box('Roof_'+side,(4.93,10.4,.065),(s*2.44,0,4.91),'panel','Roof','roof')
    obj.rotation_euler.y=s*math.atan(.55/4.82)
for j,y in enumerate([-3.5,-1,1.5,4],1):
    for k,x in enumerate([-2,2],1):
        beam(f'Hanger_{j}_{k}',(x,y,4.55),(x,y,4.12),.018,.018,'black','Utilities')
        box(f'LED_Housing_{j}_{k}',(1.4,.19,.08),(x,y,4.08),'metal','Utilities','fixture')
        box(f'LED_Diffuser_{j}_{k}',(1.32,.14,.02),(x,y,4.03),'lamp','Utilities','fixture')
        light(f'LED_Light_{j}_{k}',(x,y,3.98),100,1.6)
for j,z in enumerate([2.65,2.82],1):
    beam(f'Service_Pipe_{j}',(-4.65,-4.5,z),(-4.65,4.5,z),.055,.055,'panel','Utilities','pipe')
for i,y in enumerate([-3.7,3.7],1):
    cyl(f'Vent_Duct_{i}',.16,4.35,(4.55,y,2.175),'panel','Utilities','pipe')
    for j,z in enumerate([.4,1.4,2.4,3.4,4.15],1):
        cyl(f'Duct_Collar_{i}_{j}',.18,.07,(4.55,y,z),'steel','Utilities','pipe')
box('Roller_Shutter',(2.8,.08,2.65),(0,4.93,1.325),'panel','Architecture','door')
for j in range(17):
    box(f'Shutter_Slat_{j}',(2.76,.025,.018),(0,4.877,.1+j*.15),'steel','Architecture','door_detail')
label('Factory_Title','G1  /  ASSEMBLY HALL',(0,4.83,3.45),.26,'white')
label('Bay_Label','BAY  01',(0,4.82,2.95),.16,'yellow')
for i,x in enumerate([-3.15,3.15],1):
    for j,dx in enumerate([-1.05,1.05]):
        box(f'Workcell_{i}_Edge_{j}',(.035,2.5,.002),(x+dx,2.65,.001),'yellow','Floor_Markings','marking')
    box(f'Workcell_{i}_Front',(2.1,.035,.002),(x,1.4,.001),'yellow','Floor_Markings','marking')
    label(f'Machine_Label_{i}',f'CNC  0{i}',(x,2.267,1.66),.11,'white')
    box(f'Machine_{i}_Beacon',(.08,.08,.16),(x+.55,2.8,1.88),'orange','Machines','display')
    for j in range(2):
        box(f'Workpiece_{i}_{j}',(.24,.22,.06),(x-.35+j*.5,0,.83),'panel','Props','workpiece')
    cyl(f'Barrel_{i}',.24,.72,(x,4.2,.36),'orange','Props')
    for j,z in enumerate([.09,.61]): cyl(f'Barrel_Band_{i}_{j}',.247,.035,(x,4.2,z),'steel','Props')
for i,(x,y) in enumerate([(-4.1,-3.9),(3.5,-4.1)],1):
    box(f'Pallet_{i}',(.8,.65,.12),(x,y,.06),'wood','Props','pallet')
    box(f'Crate_{i}',(.68,.54,.55),(x,y,.395),'crate','Props','crate')
    for j,dx in enumerate([-.24,.24]): box(f'Crate_Strap_{i}_{j}',(.025,.55,.012),(x+dx,y,.676),'metal','Props','strap')
cyl('Fire_Extinguisher',.09,.55,(-4.5,-4.3,.43),'red','Props')
def worker(index,x,y,angle=0,pose='bench'):
    root=group(f'Worker_{index:02}','Workers','worker')
    pieces=[]
    def part(name,size,loc,mat):
        obj=box(f'Worker_{index:02}_{name}',size,loc,mat,'Workers','human')
        pieces.append(obj)
        return obj
    def limb(name,a,b,w,d,mat):
        obj=beam(f'Worker_{index:02}_{name}',a,b,w,d,mat,'Workers','human')
        pieces.append(obj)
    for side,s in [('L',-1),('R',1)]:
        part('Boot_'+side,(.14,.28,.12),(s*.13,-.045,.06),'black')
        limb('Shin_'+side,(s*.13,0,.12),(s*.13,.01,.52),.12,.14,'uniform')
        limb('Thigh_'+side,(s*.13,.01,.52),(s*.11,0,.91),.15,.17,'uniform')
    part('Torso',(.43,.24,.48),(0,0,1.15),'uniform')
    part('Vest',(.39,.025,.34),(0,-.135,1.17),'orange')
    for s in [-1,1]: part('Reflector_'+str(s),(.045,.008,.3),(s*.12,-.151,1.17),'yellow')
    limb('Neck',(0,0,1.39),(0,0,1.46),.1,.1,'skin')
    bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=8,radius=1,location=(0,0,1.59))
    head=bpy.context.object
    head.name=f'Worker_{index:02}_Head'
    for c in list(head.users_collection): c.objects.unlink(head)
    link(head,'Workers','human')
    head.scale=(.115,.1,.145)
    head.data.materials.append(mats['skin'])
    bpy.ops.object.transform_apply(location=False,rotation=False,scale=True)
    pieces.append(head)
    pieces.append(cyl(f'Worker_{index:02}_Hardhat',.15,.10,(0,0,1.73),'yellow','Workers','human'))
    for side,s in [('L',-1),('R',1)]:
        elbow=(s*.29,-.16,1.02 if pose=='bench' else 1.18)
        hand=(s*.19,-.43,.88 if pose=='bench' else 1.35)
        limb('Arm_'+side,(s*.23,0,1.32),elbow,.11,.12,'uniform')
        limb('Forearm_'+side,elbow,hand,.085,.09,'uniform')
        part('Glove_'+side,(.09,.1,.09),hand,'skin')
    for obj in pieces: obj.parent=root
    root.location=(x,y,0)
    root.rotation_euler.z=angle
    root['pose']=pose
worker(1,-3.15,.75)
worker(2,3.15,.75)
worker(3,-3.15,1.9,math.pi,'panel')
# Convert labels into geometry so USD keeps the lettering.
for obj in list(scene.objects):
    if obj.type=='FONT':
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active=obj
        bpy.ops.object.convert(target='MESH')
cam.location=(.55,-4.65,1.85)
cam.rotation_euler=(Vector((0,2,2.1))-cam.location).to_track_quat('-Z','Y').to_euler()
cam.data.type='PERSP'
cam.data.lens=20
cam.name='Interior_Camera'
scene.render.resolution_x=1600
scene.render.resolution_y=1000
scene.render.filepath=str(OUT/'factory_interior.png')
overview_data=bpy.data.cameras.new('Overview_Industrial')
overview=bpy.data.objects.new('Overview_Industrial',overview_data)
link(overview,'Presentation','camera')
overview.location=(11,-15,24)
overview.rotation_euler=(Vector((0,0,1))-overview.location).to_track_quat('-Z','Y').to_euler()
overview_data.type='ORTHO'
overview_data.ortho_scale=18
bpy.context.view_layer.update()
def bounds(obj):
    members=[obj] if obj.type=='MESH' else list(obj.children_recursive)
    pts=[m.matrix_world@Vector(c) for m in members if m.type=='MESH' for c in m.bound_box]
    return ([min(p[k] for p in pts) for k in range(3)],[max(p[k] for p in pts) for k in range(3)]) if pts else None
meshes=[o for o in scene.objects if o.type=='MESH']
errors=[]
for obj in meshes:
    if not obj.data.vertices or not obj.data.polygons: errors.append('empty mesh '+obj.name)
    if any(abs(v-1)>1e-5 for v in obj.scale): errors.append('unapplied scale '+obj.name)
    if not all(math.isfinite(v) for row in obj.matrix_world for v in row): errors.append('invalid transform '+obj.name)
    lo,hi=bounds(obj)
    if obj.get('asset_type')=='floor': continue
    if lo[2]<2.2 and hi[2]>0 and lo[0]<1 and hi[0]>-1 and lo[1]<1 and hi[1]>-1: errors.append('spawn obstruction '+obj.name)
    if obj.get('asset_type') not in ['marking','structure','roof','door','door_detail','wall'] and lo[2]<2.2 and hi[2]>.02 and lo[0]<1.1 and hi[0]>-1.1 and lo[1]<4.8 and hi[1]>-4.9: errors.append('aisle obstruction '+obj.name)
report={'errors':errors,'mesh_count':len(meshes),'polygons':sum(len(o.data.polygons) for o in meshes),'worker_count':3,'worker_animation':False,'aisle_width_m':2.2,'spawn_clear_m':[2,2],'wall_panel_height_m':3,'upper_wall_height_m':4.6,'ridge_height_m':5.15,'intersection_note':'Human joints, steel and pipe connections intentionally intersect. Checks cover empty mesh, scale, finite transforms, spawn and aisle; no global no-intersection claim.','physics_note':'Static colliders not configured in Isaac Sim.'}
backup(OUT/'validation_report.json')
(OUT/'validation_report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
if errors: raise RuntimeError(json.dumps(report))
objects=[]
for obj in sorted(scene.objects,key=lambda o:o.name):
    bb=bounds(obj)
    objects.append({'name':obj.name,'type':obj.get('asset_type',obj.type),'blender_type':obj.type,'location_m':list(obj.matrix_world.translation),'dimensions_m':[bb[1][i]-bb[0][i] for i in range(3)] if bb else [0,0,0],'bounds_world_m':bb,'scale':list(obj.scale),'parent':obj.parent.name if obj.parent else None,'collection':[c.name for c in obj.users_collection]})
backup(OUT/'object_manifest.json')
(OUT/'object_manifest.json').write_text(json.dumps({'units':'meters','up_axis':'Z','objects':objects},indent=2),encoding='utf-8')
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D': area.spaces.active.region_3d.view_perspective='CAMERA'
backup(BLEND)
bpy.ops.wm.save_as_mainfile(filepath=str(BLEND))
backup(OUT/'factory.usd')
result=bpy.ops.wm.usd_export(filepath=str(OUT/'factory.usd'),export_animation=False,export_lights=True,export_cameras=False,root_prim_path='/Factory',meters_per_unit=1,convert_orientation=False,export_materials=True)
assert 'FINISHED' in result
backup(OUT/'export_status.txt')
(OUT/'export_status.txt').write_text('USD revision 2 exported; static worker meshes; colliders pending.',encoding='utf-8')
backup(OUT/'factory_interior.png')
bpy.ops.render.render(write_still=True)
scene.camera=overview
scene.render.resolution_x=1400
scene.render.resolution_y=1400
scene.render.filepath=str(OUT/'factory_overview.png')
roof_objs=list(collections['Roof'].objects)
for obj in roof_objs: obj.hide_render=True
backup(OUT/'factory_overview.png')
bpy.ops.render.render(write_still=True)
for obj in roof_objs: obj.hide_render=False
scene.camera=cam
scene.render.resolution_x=1600
scene.render.resolution_y=1000
scene.render.filepath=str(OUT/'factory_interior.png')
print(json.dumps(report,indent=2))

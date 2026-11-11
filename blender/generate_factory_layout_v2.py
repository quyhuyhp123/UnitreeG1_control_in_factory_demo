r"""Rebuild the reference-derived factory only; never launches Isaac Sim.
Run: D:\Blender\blender.exe -b --python D:\unitree_g1\blender\generate_factory_layout_v2.py
"""
import bpy, math, json, time, traceback
from pathlib import Path
from collections import deque
from mathutils import Vector

ROOT=Path(r'D:\unitree_g1')
OUT=ROOT/'assets/factory/factory_layout_v2'
C=json.loads((OUT/'factory_dimensions.json').read_text(encoding='utf-8'))
START=time.perf_counter()
def write(name,data): (OUT/name).write_text(json.dumps(data,indent=2),encoding='utf-8')
def xy(p):
    a,b,c,d=C['reference_pixel_bounds']
    return [(p[0]-a)/(c-a)*C['width_m']-C['width_m']/2,C['length_m']/2-(p[1]-b)/(d-b)*C['length_m']]
def rect(p):
    a=xy(p[:2]); b=xy(p[2:]); return [[a[0],b[1]],[b[0],b[1]],[b[0],a[1]],[a[0],a[1]]]
def inside(p,poly):
    yes=False; x,y=p
    for a,b in zip(poly,poly[1:]+poly[:1]):
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]: yes=not yes
    return yes
def distance_segment(p,a,b):
    d=[b[i]-a[i] for i in (0,1)]; n=sum(v*v for v in d)
    t=max(0,min(1,sum((p[i]-a[i])*d[i] for i in (0,1))/n)) if n else 0
    return math.dist(p,[a[i]+t*d[i] for i in (0,1)])

# A fresh scene in this background Blender process; existing .blend is untouched.
bpy.ops.wm.read_factory_settings(use_empty=True)
scene=bpy.context.scene; scene.name='Factory_Layout_V2'
scene.unit_settings.system='METRIC'; scene.unit_settings.scale_length=1
root=bpy.data.collections.new('Factory_Root'); scene.collection.children.link(root)
COL={}
for name in ['Building','Floor','Outer_Machines','Inner_Machine_Island_Left','Inner_Machine_Island_Right','Work_Areas','Humans','Navigation_Visuals','Navigation_Markers','Collision_Proxies','Lighting','Cameras']:
    col=bpy.data.collections.new(name); root.children.link(col); COL[name]=col
MAT={}
for name,rgb in {'concrete':(.32,.36,.38),'wall':(.73,.77,.79),'green':(.025,.48,.12),'yellow':(1,.77,.015),'island':(.84,.85,.80),'machine':(.65,.045,.045),'steel':(.12,.16,.19),'panel':(.57,.64,.67),'screen':(.025,.16,.20),'desk':(.05,.4,.67),'paper':(.96,.95,.87),'skin':(.59,.35,.21),'uniform':(.045,.14,.28),'helmet':(.96,.55,.025),'lamp':(.2,.9,.35)}.items():
    m=bpy.data.materials.new(name); m.diffuse_color=(*rgb,1); m.use_nodes=True
    bsdf=m.node_tree.nodes.get('Principled BSDF'); bsdf.inputs['Base Color'].default_value=(*rgb,1); bsdf.inputs['Roughness'].default_value=.65
    MAT[name]=m
CACHE={}; META={}
def link(obj,col,semantic,obstacle=False,walkable=False,interactable=False,group=None):
    COL[col].objects.link(obj)
    obj['semantic_class']=semantic; obj['object_id']=obj.name; obj['is_static']=True
    obj['is_navigation_obstacle']=bool(obstacle); obj['is_walkable']=bool(walkable); obj['is_interactable']=bool(interactable)
    if group: obj['assembly_id']=group
    META[obj.name]={'class':semantic,'obstacle':obstacle,'group':group,'proxy':col=='Collision_Proxies'}
    return obj
def box(name,dims,pos,mat,col,semantic,obstacle=False,group=None):
    key=(tuple(round(float(d),6) for d in dims),mat)
    if key not in CACHE:
        x,y,z=[float(d)/2 for d in dims]
        verts=[(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),(-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
        mesh=bpy.data.meshes.new('SharedBox_%03d'%len(CACHE))
        mesh.from_pydata(verts,[],[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)])
        mesh.materials.append(MAT[mat]); mesh.update(); CACHE[key]=mesh
    obj=bpy.data.objects.new(name,CACHE[key]); obj.location=pos
    return link(obj,col,semantic,obstacle,semantic=='walkable_area',group=group)
def empty(name,pos,col='Navigation_Markers',semantic='marker'):
    obj=bpy.data.objects.new(name,None); obj.location=pos; obj.empty_display_type='ARROWS'; obj.empty_display_size=.4
    return link(obj,col,semantic)
def proxy(name,dims,pos,semantic,assembly):
    obj=box('COL_'+name,dims,pos,'steel','Collision_Proxies',semantic,True,assembly)
    obj.hide_render=True; obj.hide_set(True); obj.display_type='WIRE'; obj['collision_recommendation']='static_box; activate after Isaac import'
    return obj
def rod(name,a,b,width,mat,col,semantic,group=None):
    a,b=Vector(a),Vector(b); o=box(name,(width,width,(b-a).length),(a+b)/2,mat,col,semantic,False,group)
    o.rotation_euler=(b-a).to_track_quat('Z','Y').to_euler(); return o
def ball(name,r,pos,mat,col,semantic,group=None):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=12,ring_count=6,radius=r,location=pos)
    obj=bpy.context.object; obj.name=name
    for c in list(obj.users_collection): c.objects.unlink(obj)
    obj.data.materials.append(MAT[mat]); return link(obj,col,semantic,group=group)

W,L,H=C['width_m'],C['length_m'],C['wall_height_m']; wall=C['wall_thickness_m']; thick=C['floor_thickness_m']
box('Factory_Floor',(W,L,thick),(0,0,-thick/2),'concrete','Floor','floor')
proxy('Floor',(W,L,thick),(0,0,-thick/2),'floor','Factory_Floor')
walls=[('North',(W,wall,H),(0,L/2+wall/2,H/2)),('South',(W,wall,H),(0,-L/2-wall/2,H/2)),('East',(wall,L,H),(W/2+wall/2,0,H/2)),('West',(wall,L-C['entrance_width_m'],H),(-W/2-wall/2,C['entrance_width_m']/2,H/2))]
for name,dim,pos in walls:
    o=box('Factory_Wall_'+name,dim,pos,'wall','Building','wall',True); o['hide_in_top_view']=True
    proxy('Wall_'+name,dim,pos,'wall',o.name)
entry=empty('Factory_Entrance',(-W/2,-L/2+C['entrance_width_m']/2,0),'Building','entrance'); entry['width_m']=C['entrance_width_m']
enclosure=C['building_enclosure']; roof_t=enclosure['roof_thickness_m']; door_h=enclosure['door_height_m']
roof=box('Factory_Roof',(W+2*wall,L+2*wall,roof_t),(0,0,H+roof_t/2),'wall','Building','roof')
roof['hide_in_top_view']=True
proxy('Roof',(W+2*wall,L+2*wall,roof_t),(0,0,H+roof_t/2),'roof',roof.name)
door_y=entry.location.y; frame=enclosure['door_frame_m']; gap=C['entrance_width_m']
for sign in (-1,1):
    pos=(-W/2-wall/2,door_y+sign*(gap/2+frame/2),door_h/2)
    box('Entrance_Jamb_'+str(sign),(.25,frame,door_h),pos,'steel','Building','wall')
box('Entrance_Lintel',(.25,gap,H-door_h),(-W/2-wall/2,door_y,(H+door_h)/2),'wall','Building','wall',True)
proxy('Entrance_Lintel',(.25,gap,H-door_h),(-W/2-wall/2,door_y,(H+door_h)/2),'wall','Entrance_Lintel')
# Raised roller shutter leaves the full 2.5 m entrance unobstructed.
leaf=box('Entrance_Roller_Shutter',(.18,gap,.55),(-W/2-.22,door_y,door_h+.35),'panel','Building','door')
leaf['door_state']='OPEN_RAISED'; leaf['is_interactable']=True
proxy(leaf.name,(.18,gap,.55),list(leaf.location),'door',leaf.name)
for sign in (-1,1):
    box('Entrance_Door_Track_'+str(sign),(.08,.06,door_h),(-W/2-.18,door_y+sign*(gap/2+.035),door_h/2),'steel','Building','door')
outer=[xy(p) for p in C['walkable_outer_pixels']]; holes=[list(reversed(rect(p))) for p in C['island_holes_pixels']]
def allowed(p): return inside(p,outer) and not any(inside(p,h) for h in holes)
for i,h in enumerate(holes):
    xs=[p[0] for p in h]; ys=[p[1] for p in h]
    box('Island_Base_%02d'%(i+1),(max(xs)-min(xs),max(ys)-min(ys),.004),((max(xs)+min(xs))/2,(max(ys)+min(ys))/2,.002),'island','Floor','floor')
# Non-overlapping rectangles tessellate the polygon with two true holes.
xs=sorted({p[0] for poly in [outer]+holes for p in poly}); ys=sorted({p[1] for poly in [outer]+holes for p in poly})
verts=[]; faces=[]
for x0,x1 in zip(xs,xs[1:]):
    for y0,y1 in zip(ys,ys[1:]):
        if allowed(((x0+x1)/2,(y0+y1)/2)):
            n=len(verts); verts.extend([(x0,y0,C['overlay_height_m']),(x1,y0,C['overlay_height_m']),(x1,y1,C['overlay_height_m']),(x0,y1,C['overlay_height_m'])]); faces.append((n,n+1,n+2,n+3))
mesh=bpy.data.meshes.new('WalkablePolygonWithHoles'); mesh.from_pydata(verts,[],faces); mesh.materials.append(MAT['green'])
link(bpy.data.objects.new('Robot_Walkable_Area',mesh),'Navigation_Visuals','walkable_area',walkable=True)
for name,polys,mat in [('Robot_Navigation_Boundaries',[outer]+holes,'yellow')]:
    vv=[]; ff=[]
    for poly in polys:
        for a,b in zip(poly,poly[1:]+poly[:1]):
            dx,dy=b[0]-a[0],b[1]-a[1]; length=math.hypot(dx,dy); nx,ny=-dy/length*C['yellow_line_width_m']/2,dx/length*C['yellow_line_width_m']/2
            n=len(vv); z=C['overlay_height_m']+.003
            vv.extend([(a[0]+nx,a[1]+ny,z),(a[0]-nx,a[1]-ny,z),(b[0]-nx,b[1]-ny,z),(b[0]+nx,b[1]+ny,z)]); ff.append((n,n+1,n+2,n+3))
    m=bpy.data.meshes.new(name); m.from_pydata(vv,[],ff); m.materials.append(MAT[mat]); link(bpy.data.objects.new(name,m),'Navigation_Visuals','navigation_boundary')
forbid=empty('Robot_Forbidden_Areas',(0,0,0),'Navigation_Visuals','forbidden_area'); forbid['polygons_json']=json.dumps(holes)
spawn=empty('Robot_Entry_Spawn',C['spawn_m']); spawn.rotation_euler.z=math.radians(C['spawn_yaw_deg'])
for name,pos in C['patrol_markers_m'].items(): empty(name,pos)

machine_bounds=[]
for name,pixels in C['machine_bounds_pixels'].items():
    poly=rect(pixels); x0,y0=poly[0]; x1,y1=poly[2]; dx,dy=x1-x0,y1-y0; x,y=(x0+x1)/2,(y0+y1)/2
    dx,dy,uniform_height=C['uniform_machine_dimensions_m']['outer' if 'Outer' in name else 'inner']
    x0,x1=x-dx/2,x+dx/2; y0,y1=y-dy/2,y+dy/2
    col='Outer_Machines' if 'Outer' in name else ('Inner_Machine_Island_Left' if 'Left' in name else 'Inner_Machine_Island_Right')
    parent=empty(name,(x,y,0),col,'machine'); parent['is_navigation_obstacle']=True
    ht=uniform_height; facing=1 if 'Left' in name else -1
    parts=[('Base',(dx,dy,.18),(x,y,.09),'steel'),('Main_Body',(dx-.08,dy-.08,ht-.18),(x,y,(ht+.18)/2),'machine'),
        ('Door',(.035,dy*.62,ht*.67),(x+facing*(dx/2-.02),y,ht*.51),'panel'),
        ('Control_Panel',(.08,min(.50,dy*.25),.55),(x+facing*(dx/2+.04),y+dy*.27,1.25),'steel'),
        ('HMI',(.015,min(.34,dy*.16),.25),(x+facing*(dx/2+.087),y+dy*.27,1.33),'screen'),
        ('Status_Light',(.1,.1,.17),(x+facing*(dx/2-.16),y+dy/2-.2,ht+.085),'lamp')]
    for suffix,dim,pos,mat in parts:
        o=box(name+'_'+suffix,dim,pos,mat,col,'machine',False,name)
        o.parent=parent; o.location=Vector(pos)-parent.location
    proxy(name,(dx+.20,dy,ht+.18),(x,y,(ht+.18)/2),'machine',name)
    machine_bounds.append({'id':name,'semantic_class':'machine','bounds':[[x0-.1,y0,0],[x1+.1,y1,ht+.18]]})
    # Outside operators use the short end within the grey equipment strip;
    # island operators remain inside their forbidden island, outside the aisle.
    if 'Outer' in name: wx,wy,yaw=x,y-dy/2-C['standing_worker']['machine_gap_m'],0
    else: wx,wy,yaw=x+facing*(dx/2+C['standing_worker']['machine_gap_m']),y,math.pi/2 if facing>0 else -math.pi/2
    person='Operator_'+name.removeprefix('Machine_')
    parts=[]
    parts.append(box(person+'_Torso',(.42,.26,.53),(wx,wy,1.16),'uniform','Humans','person',False,person))
    parts.append(box(person+'_Pelvis',(.36,.28,.18),(wx,wy,.83),'uniform','Humans','person',False,person))
    parts.append(ball(person+'_Head',.135,(wx,wy,1.61),'skin','Humans','person',person))
    parts.append(ball(person+'_Helmet',.145,(wx,wy,1.68),'helmet','Humans','person',person))
    for side in (-1,1):
        sx=wx+side*.12
        parts.append(rod(person+'_Thigh_'+str(side),(sx,wy,.78),(sx,wy,.47),.13,'uniform','Humans','person',person))
        parts.append(rod(person+'_Shin_'+str(side),(sx,wy,.47),(sx,wy,.13),.11,'uniform','Humans','person',person))
        parts.append(box(person+'_Shoe_'+str(side),(.14,.26,.12),(sx,wy+.05,.07),'steel','Humans','person',False,person))
        parts.append(rod(person+'_UpperArm_'+str(side),(wx+side*.25,wy,1.34),(wx+side*.28,wy+.10,1.05),.09,'uniform','Humans','person',person))
        parts.append(rod(person+'_Forearm_'+str(side),(wx+side*.28,wy+.10,1.05),(wx+side*.25,wy+.31,1.12),.07,'skin','Humans','person',person))
    for part in parts:
        offset=part.location-Vector((wx,wy,0)); offset.rotate(__import__('mathutils').Matrix.Rotation(yaw,3,'Z'))
        part.location=Vector((wx,wy,0))+offset; part.rotation_euler.rotate_axis('Z',yaw)
        part['works_at']=name
    proxy(person,(.58,.56,1.84),(wx,wy,.92),'person',person)

for i,p in enumerate(C['table_centers_pixels'],1):
    x,y=xy(p); group='Table_%02d'%i; dx,dy,ht=C['table_dimensions_m']
    box(group,(dx,dy,.08),(x,y,ht-.04),'desk','Work_Areas','table',False,group)
    for j,(sx,sy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
        box(group+'_Leg_%02d'%j,(.07,.07,ht-.08),(x+sx*(dx/2-.12),y+sy*(dy/2-.1),(ht-.08)/2),'steel','Work_Areas','table',False,group)
    proxy(group,(dx,dy,ht),(x,y,ht/2),'table',group)
    box(group+'_Monitor',(.72,.08,.40),(x+.68,y+.20,1.05),'steel','Work_Areas','table',False,group)
    box(group+'_Screen',(.64,.012,.32),(x+.68,y+.153,1.05),'screen','Work_Areas','table',False,group)
    box(group+'_Stand',(.12,.12,.09),(x+.68,y+.20,.845),'steel','Work_Areas','table',False,group)
    for j in range(3):
        paper=box('Document_%02d_%02d'%(i,j),(.30,.21,.004),(x-.65+j*.12,y-.05+j*.05,ht+.003+j*.005),'paper','Work_Areas','document')
        paper.rotation_euler.z=.1*j
    hx=x+C['seated_person_offset_m'][0]; hy=y+C['seated_person_offset_m'][1]
    chair='Chair_%02d'%i
    box(chair,(.48,.48,.07),(hx,hy,.47),'steel','Work_Areas','chair')
    box(chair+'_Back',(.48,.06,.50),(hx,hy-.25,.75),'steel','Work_Areas','chair')
    for j,(sx,sy) in enumerate([(-1,-1),(-1,1),(1,-1),(1,1)]):
        box(chair+'_Leg_%02d'%j,(.04,.04,.435),(hx+sx*.19,hy+sy*.19,.2175),'steel','Work_Areas','chair')
    person='Person_%02d'%i
    box(person+'_Torso',(.40,.25,.48),(hx,hy,.92),'uniform','Humans','person',False,person)
    box(person+'_Pelvis',(.37,.30,.14),(hx,hy,.61),'uniform','Humans','person',False,person)
    ball(person+'_Head',.135,(hx,hy,1.31),'skin','Humans','person',person)
    helmet=ball(person+'_Helmet',.145,(hx,hy,1.37),'helmet','Humans','person',person)
    for side in (-1,1):
        sx=hx+side*.14
        rod(person+'_Thigh_'+str(side),(sx,hy,.61),(sx,hy+.36,.57),.14,'uniform','Humans','person',person)
        rod(person+'_Shin_'+str(side),(sx,hy+.36,.54),(sx,hy+.40,.14),.11,'uniform','Humans','person',person)
        box(person+'_Shoe_'+str(side),(.12,.24,.12),(sx,hy+.46,.08),'steel','Humans','person',False,person)
        rod(person+'_Upper_Arm_'+str(side),(hx+side*.24,hy,1.08),(hx+side*.27,hy+.17,.84),.09,'uniform','Humans','person',person)
        rod(person+'_Forearm_'+str(side),(hx+side*.27,hy+.17,.84),(hx+side*.25,hy+.40,.84),.07,'skin','Humans','person',person)
    proxy(person,(.62,.85,1.52),(hx,hy+.15,.76),'person',person)
    proxy(chair,(.55,.57,1.0),(hx,hy, .5),'chair',chair)

# Basic lighting, no textures, no roof or sensors.
scene.world=bpy.data.worlds.new('Factory_World'); scene.world.color=(.32,.32,.32)
for i,(x,y) in enumerate([(-5,-8),(5,-8),(-5,6),(5,6)]):
    data=bpy.data.lights.new('Area_%02d'%i,'AREA'); data.energy=650; data.shape='DISK'; data.size=6
    obj=link(bpy.data.objects.new(data.name,data),'Lighting','light'); obj.location=(x,y,H-.1)
def camera(name,pos,target,scale):
    data=bpy.data.cameras.new(name); obj=link(bpy.data.objects.new(name,data),'Cameras','camera'); obj.location=pos
    obj.rotation_euler=(Vector(target)-obj.location).to_track_quat('-Z','Y').to_euler(); data.type='ORTHO'; data.ortho_scale=scale
    data.clip_end=200; return obj
top=camera('Camera_Top',(0,0,45),(0,0,0),56)
iso=camera('Camera_Isometric',(31,-42,39),(0,0,.5),48)
exterior=camera('Camera_Exterior',(-34,-43,30),(0,0,1),48)

# Simple whole-footprint geometry check, NOT an online planner.
radius=sum(C['footprint'].values()); res=C['validation_resolution_m']
obstacles=[]
bpy.context.view_layer.update()
for o in COL['Collision_Proxies'].objects:
    corners=[o.matrix_world@Vector(v) for v in o.bound_box]
    lo=[min(p[k] for p in corners) for k in range(3)]; hi=[max(p[k] for p in corners) for k in range(3)]
    if o.name!='COL_Floor': obstacles.append({'id':o.name,'semantic_class':o['semantic_class'],'bounds':[lo,hi]})
def safe(p,extra=0):
    p=p[:2]
    r=radius+extra
    if not allowed(p): return False
    for poly in [outer]+holes:
        if min(distance_segment(p,a,b) for a,b in zip(poly,poly[1:]+poly[:1]))<r: return False
    for row in obstacles:
        lo,hi=row['bounds']; d=math.hypot(max(lo[0]-p[0],0,p[0]-hi[0]),max(lo[1]-p[1],0,p[1]-hi[1]))
        if lo[2]>1.9 or hi[2]<.06: continue
        if d<r: return False
    return True
nx,ny=round(W/res),round(L/res)
free=set()
for j in range(ny):
    for i in range(nx):
        if safe((-W/2+(i+.5)*res,-L/2+(j+.5)*res),res*math.sqrt(2)/2): free.add((i,j))
def cell(p): return (int((p[0]+W/2)/res),int((p[1]+L/2)/res))
seen=set(); todo=deque([cell(C['spawn_m'])])
while todo:
    c=todo.popleft()
    if c in seen or c not in free: continue
    seen.add(c)
    for di,dj in [(1,0),(-1,0),(0,1),(0,-1)]: todo.append((c[0]+di,c[1]+dj))
markers={name:{'position_m':p,'footprint_safe':safe(p),'connected_to_spawn':cell(p) in seen} for name,p in C['patrol_markers_m'].items()}
nav={'version':2,'coordinate_system':{'units':'meters','up_axis':'Z','x_axis':'reference right','y_axis':'reference up','origin':'factory center'},
    'allowed_polygons':[{'id':'Connected_Green_Corridors','outer':outer,'holes':holes}],
    'forbidden_polygons':[{'id':'Machine_Island_%02d'%i,'polygon':h} for i,h in enumerate(holes,1)],
    'static_obstacles':obstacles,'robot_spawn':{'position_m':C['spawn_m'],'yaw_deg':C['spawn_yaw_deg']},
    'entrance':{'position_m':list(entry.location),'width_m':C['entrance_width_m'],'wall':'west'},
    'navigation_boundaries':{'loops':[outer]+holes,'physical_collider':False},'footprint':C['footprint'],
    'patrol_markers':markers,'validation':{'resolution_m':res,'footprint_radius_m':radius,'safe_cells':len(free),'spawn_component_cells':len(seen),'all_free_cells_connected':len(free)==len(seen),'spawn_safe':safe(C['spawn_m']),'all_markers_connected':all(r['connected_to_spawn'] for r in markers.values())},
    'integration_note':'Standalone polygon-with-holes schema. Existing navigation code does not consume this schema yet. Do not replace active navigation_map.json without a separate integration task.'}
write('navigation_allowed_areas.json',nav)

manifest=[]; triangles=0; issues=[]
for obj in scene.objects:
    if obj.type=='MESH':
        obj.data.calc_loop_triangles(); triangles+=len(obj.data.loop_triangles)
        if not obj.data.polygons: issues.append('Empty mesh: '+obj.name)
        if any(abs(s-1)>1e-6 for s in obj.scale): issues.append('Non-unit scale: '+obj.name)
        if not all(math.isfinite(v) for v in list(obj.location)+list(obj.rotation_euler)+list(obj.dimensions)): issues.append('Nonfinite: '+obj.name)
    corners=[obj.matrix_world@Vector(v) for v in obj.bound_box] if obj.type=='MESH' else [obj.matrix_world.translation]
    manifest.append({'object_name':obj.name,'semantic_class':obj['semantic_class'],'object_id':obj['object_id'],
        'is_static':obj['is_static'],'is_navigation_obstacle':obj['is_navigation_obstacle'],'is_walkable':obj['is_walkable'],'is_interactable':obj['is_interactable'],
        'position_m':list(obj.matrix_world.translation),'rotation_euler_world_rad':list(obj.matrix_world.to_euler()),'dimensions_m':list(obj.dimensions),
        'world_bounding_box_m':[[min(p[k] for p in corners) for k in range(3)],[max(p[k] for p in corners) for k in range(3)]],
        'collections':[c.name for c in obj.users_collection],'assembly_id':obj.get('assembly_id'),
        'collision_recommendation':obj.get('collision_recommendation','none; use assembly COL_ proxy' if obj.type=='MESH' else 'none'),
        'usd_prim_path':None})
write('object_manifest.json',{'objects':manifest,'semantic_note':'Custom properties are preparation metadata, NOT verified Isaac semantic labels. No PhysX APIs have been authored by this script.'})
scene.render.engine=C['preview']['engine']; scene.render.resolution_x=C['preview']['width']; scene.render.resolution_y=C['preview']['height']; scene.render.resolution_percentage=100
scene.display.shading.light='STUDIO'; scene.display.shading.color_type='MATERIAL'; scene.display.shading.show_shadows=True
scene.display.shading.show_cavity=True; scene.display.shading.cavity_type='BOTH'; scene.display.shading.show_specular_highlight=False
scene.display.shading.background_type='WORLD'; scene.world.color=(.085,.10,.13)
scene.view_settings.view_transform='Standard'
scene.camera=iso
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type=='VIEW_3D': area.spaces.active.region_3d.view_perspective='CAMERA'
blend=ROOT/'blender/factory_layout_v2.blend'
bpy.ops.wm.save_as_mainfile(filepath=str(blend))
exports={}
def export(op,kwargs,label):
    try:
        props=op.get_rna_type().properties.keys(); use={k:v for k,v in kwargs.items() if k in props}
        result=op(**use); exports[label]={'status':'passed' if 'FINISHED' in result else 'failed','options':use,'result':list(result)}
    except Exception as exc: exports[label]={'status':'failed','error':repr(exc)}; traceback.print_exc()
for obj in COL['Collision_Proxies'].objects: obj.hide_render=False; obj.hide_set(False)
export(bpy.ops.wm.usd_export,{'filepath':str(OUT/'factory_layout_v2.usd'),'selected_objects_only':False,'export_custom_properties':True,'export_materials':True,'export_uvmaps':False,'generate_preview_surface':True},'usd')
from pxr import Usd,UsdGeom
stage=Usd.Stage.Open(str(OUT/'factory_layout_v2.usd'))
for prim in stage.Traverse():
    value=prim.GetAttribute('userProperties:object_id').Get()
    if value and str(value).startswith('COL_'):
        UsdGeom.Imageable(prim).CreatePurposeAttr().Set('proxy')
        UsdGeom.Imageable(prim).CreateVisibilityAttr().Set('invisible')
stage.GetRootLayer().Save(); stage=None
for obj in COL['Collision_Proxies'].objects: obj.hide_render=True; obj.hide_set(True)
export(bpy.ops.export_scene.gltf,{'filepath':str(OUT/'factory_layout_v2.glb'),'export_format':'GLB','use_visible':True,'export_cameras':True,'export_lights':True,'export_extras':True,'export_yup':True},'glb')
for name,cam in [('preview_top.png',top),('preview_isometric.png',iso),('preview_exterior.png',exterior)]:
    bpy.data.objects['Factory_Roof'].hide_render=name!='preview_exterior.png'
    scene.display.shading.show_shadows=name!='preview_top.png'
    # Preview-only cutaway; the saved source and exports retain all walls.
    for wallname in ['Factory_Wall_South','Factory_Wall_East']:
        bpy.data.objects[wallname].hide_render=name=='preview_isometric.png'
    scene.camera=cam; scene.render.filepath=str(OUT/name)
    bpy.ops.render.render(write_still=True)
scene.camera=iso
for wallname in ['Factory_Wall_South','Factory_Wall_East']: bpy.data.objects[wallname].hide_render=False
bpy.data.objects['Factory_Roof'].hide_render=False
report={'blender_version':bpy.app.version_string,'machine_count':len(C['machine_bounds_pixels']),'table_count':2,'seated_people':2,
    'standing_operators':14,'total_people':16,'uniform_machine_dimensions_m':C['uniform_machine_dimensions_m'],'enclosure':'four walls, roof, lintel, open raised roller shutter',
    'objects':len(scene.objects),'mesh_objects':sum(o.type=='MESH' for o in scene.objects),'triangles_including_proxies':triangles,
    'materials':len(MAT),'texture_images':0,'shared_box_meshes':len(CACHE),'collision_proxy_count':len(COL['Collision_Proxies'].objects),
    'geometry_issues':issues,'navigation':nav['validation'],'exports':exports,'build_seconds':time.perf_counter()-START,
    'corridor_widths_m':{'left_longitudinal':(435-384)/627*W,'central_longitudinal':(654-603)/627*W,'right_longitudinal':(872-823)/627*W,'north':(210-167)/530*L,'south':(647-540)/530*L},
    'files_bytes':{p.name:p.stat().st_size for p in [blend,OUT/'factory_layout_v2.usd',OUT/'factory_layout_v2.glb',OUT/'preview_top.png',OUT/'preview_isometric.png'] if p.exists()},
    'physx_and_isaac_semantics':'NOT TESTED; not launched; collider proxies are recommendations only'}
write('build_validation.json',report)
print('FACTORY_V2_RESULT '+json.dumps(report),flush=True)
if issues or not nav['validation']['all_markers_connected'] or exports.get('usd',{}).get('status')!='passed': raise RuntimeError('Factory validation failed; inspect build_validation.json')

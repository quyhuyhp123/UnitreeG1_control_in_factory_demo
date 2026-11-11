"""Isaac omni.ui patrol panel plus non-colliding USD map overlays."""
import math,time
from navigation.local_planner import pose


class PatrolPanel:
    def __init__(self,nav):
        import omni.ui as ui
        self.nav=nav; self.runtime=nav.runtime; self.last_update=0; self.visible_map=True
        self.window=ui.Window('G1 PATROL NAVIGATION',width=365,height=600)
        self.window.position_x=16; self.window.position_y=64
        self.runtime.controller_window.visible=False
        self.fields={}; self.points={}
        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=4):
                    ui.Label('Locomotion RL | SIM GROUND TRUTH',height=22)
                    ui.Label('SONIC disabled: full PhysX backend not validated. No sensor AI.',word_wrap=True,height=30)
                    self.mode=ui.ComboBox(0,'MANUAL','AUTO PATROL',height=24)
                    ui.Button('Apply Mode / explicit Manual takeover',height=24,clicked_fn=lambda:self.call(self.apply_mode))
                    ui.Button('Backend details',height=22,clicked_fn=lambda:setattr(self.runtime.controller_window,'visible',not self.runtime.controller_window.visible))
                    if hasattr(nav,'route_manager'):
                        with ui.CollapsableFrame('ROUTE SELECTION',collapsed=False):
                            with ui.VStack(spacing=3):
                                self.route_choice=ui.ComboBox(0,'Route 1 - Blue / Center','Route 2 - Brown / Right','Alternate Routes',height=24)
                                self.first_route=ui.ComboBox(0,'Alternate starts Route 1','Alternate starts Route 2',height=22)
                                ui.Button('Apply route (STOP + settle first)',height=24,clicked_fn=lambda:self.call(self.apply_route))
                                with ui.HStack(height=24):
                                    ui.Button('PREVIEW ROUTE',clicked_fn=lambda:self.call(self.preview_route))
                                    ui.Button('VALIDATE ROUTE',clicked_fn=lambda:self.call(self.preview_route))
                                ui.Button('Reset route configuration',height=22,clicked_fn=lambda:self.call(self.reset_routes))
                    with ui.CollapsableFrame('A / B coordinates (meters; heading degrees)',collapsed=False,visible=not hasattr(nav,'route_manager')):
                        with ui.VStack(spacing=3):
                            for key,value in [('A',nav.a),('B',nav.b)]:
                                ui.Label(key+(' (UNSET until Apply A/B)' if value is None else ''),height=18)
                                with ui.HStack(height=22):
                                    row=[]
                                    for v in (value[:2] if value else [0,0]):
                                        field=ui.FloatField(); field.model.set_value(v); row.append(field.model)
                                    heading=ui.FloatField(); heading.model.set_value(math.degrees(value[2]) if value and value[2] is not None else 0)
                                    enabled=ui.CheckBox(width=20); enabled.model.set_value(bool(value and value[2] is not None))
                                    row.extend([heading.model,enabled.model]); self.points[key]=row
                                ui.Button('Set '+key+' from Robot',height=22,clicked_fn=lambda k=key:self.call(lambda:self.set_current(k)))
                            ui.Label('Heading checkbox enables final heading tolerance.',height=20)
                            ui.Button('Apply A/B + Validate Route',height=25,clicked_fn=lambda:self.call(self.apply_points))
                    with ui.CollapsableFrame('Schedule / limits',collapsed=True):
                        with ui.VStack(spacing=3):
                            for label,key in [('Start delay s','start_delay_seconds'),('Wait after return A (clock s)','repeat_interval_seconds'),('Dwell B s','dwell_at_b_seconds'),('Movement timeout s','movement_timeout_seconds'),('Obstacle wait timeout s','obstacle_wait_timeout_seconds'),('Total A-B-A cycles (0 unlimited)','max_cycles')]:
                                with ui.HStack(height=22):
                                    ui.Label(label,width=210); field=ui.FloatField(); field.model.set_value(nav.scheduler.config[key]); self.fields[key]=field.model
                            with ui.HStack(height=22):
                                ui.Label('Repeat enabled',width=210); self.repeat=ui.CheckBox(); self.repeat.model.set_value(nav.scheduler.config['repeat_enabled'])
                            self.clock=ui.ComboBox(0 if nav.scheduler.config['clock_mode']=='simulation' else 1,'Simulation Time','Wall Time',height=22)
                            with ui.HStack(height=22):
                                ui.Label(f"Max speed m/s (0.12 - {getattr(nav,'speed_limit',.35):g})",width=210); self.speed=ui.FloatField(); self.speed.model.set_value(nav.config['local']['max_speed_m_s'])
                            ui.Label(('Each cycle = complete route back to ENTRY. ' if hasattr(nav,'route_manager') else 'Each cycle = A > B > A. ')+'Repeat must be enabled. Wait uses selected clock; simulation seconds may run slower than real time.',word_wrap=True,height=44)
                            if hasattr(nav,'speed_limit'):
                                with ui.HStack(height=22):
                                    for value in sorted(set([v for v in (.22,.25,.30,nav.speed_limit) if v<=nav.speed_limit])):
                                        ui.Button(f'{value:g} m/s',clicked_fn=lambda v=value:self.speed.model.set_value(v))
                            ui.Button('Apply schedule / speed (while waiting)',height=24,clicked_fn=lambda:self.call(self.apply_schedule))
                    with ui.HStack(height=26):
                        ui.Button('START PATROL',clicked_fn=lambda:self.call(self.start))
                        ui.Button('STOP',clicked_fn=lambda:self.call(nav.stop))
                    with ui.HStack(height=24):
                        ui.Button('PAUSE Auto',clicked_fn=lambda:self.call(nav.pause))
                        ui.Button('RESUME Auto',clicked_fn=lambda:self.call(nav.resume))
                    with ui.HStack(height=24):
                        ui.Button('RETURN TO ENTRY' if hasattr(nav,'route_manager') else 'RETURN TO A',clicked_fn=lambda:self.call(nav.return_to_a))
                        ui.Button('RESET ROBOT',clicked_fn=self.runtime.keys.request_reset)
                    ui.Button('Acknowledge FAULT (does not restart)',height=24,clicked_fn=lambda:self.call(nav.acknowledge_fault))
                    with ui.HStack(height=24):
                        ui.Button('Save config',clicked_fn=lambda:self.call(self.save))
                        ui.Button('Load config',clicked_fn=lambda:self.call(self.load))
                    with ui.HStack(height=24):
                        ui.Button('Top View',clicked_fn=lambda:self.call(self.top_view))
                        ui.Button('Show / Hide Path',clicked_fn=self.toggle_map)
                    self.monitor=ui.Label('',height=330 if hasattr(nav,'route_manager') else 210,word_wrap=True)
                    self.message=ui.Label(('Select and validate a route. ' if hasattr(nav,'route_manager') else 'B must be selected explicitly. ')+'Stop keys: '+','.join(k for k,v in self.runtime.keys.mapping.items() if v=='stop')+'. Camera follow: '+','.join(k for k,v in self.runtime.keys.mapping.items() if v=='follow_camera')+'. Resume Auto is explicit.',height=90,word_wrap=True)
        if hasattr(nav,'route_manager'): self.sync()
        self.draw()

    def call(self,fn):
        try:
            fn(); self.message.text=self.nav.reason
        except Exception as error: self.message.text=str(error)
    def apply_mode(self):
        self.nav.set_mode('AUTO PATROL' if self.mode.model.get_item_value_model().as_int else 'MANUAL')
    def apply_route(self):
        modes=['route_1_blue','route_2_brown','alternate']; first=['route_1_blue','route_2_brown']
        self.nav.select_route(modes[self.route_choice.model.get_item_value_model().as_int],first[self.first_route.model.get_item_value_model().as_int])
        self.nav.reason='Selected '+self.nav.selected_route()+'; validate before start'
    def preview_route(self): self.nav.validate_route(); self.visible_map=True; self.draw()
    def reset_routes(self): self.nav.reset_route_configuration(); self.sync(); self.draw()
    def apply_points(self):
        if hasattr(self.nav,'route_manager'): raise ValueError('Multi-route uses mandatory nodes; edit patrol_routes.json while stopped')
        if self.nav.mode!='MANUAL' and self.nav.scheduler.state not in ('IDLE','STOPPED'): raise ValueError('STOP before editing points')
        values={key:[row[0].as_float,row[1].as_float,math.radians(row[2].as_float) if row[3].as_bool else None] for key,row in self.points.items()}
        self.nav.validate_points(values['A'],values['B'])
        self.nav.a=values['A']; self.nav.b=values['B']; self.draw()
    def set_current(self,key):
        if hasattr(self.nav,'route_manager'): raise ValueError('Route entry/checkpoints come from verified graph, not A/B capture')
        value=list(pose(self.runtime.policy.feedback.read_feedback())); self.nav.set_point(key,value)
        self.sync()
    def sync(self):
        if hasattr(self,'route_choice'):
            self.route_choice.model.get_item_value_model().set_value(['route_1_blue','route_2_brown','alternate'].index(self.nav.route_mode))
            self.first_route.model.get_item_value_model().set_value(0 if self.nav.alternate_first=='route_1_blue' else 1)
        for key,value in [('A',self.nav.a),('B',self.nav.b)]:
            if value:
                row=self.points[key]; row[0].set_value(value[0]); row[1].set_value(value[1]); row[2].set_value(math.degrees(value[2]) if value[2] is not None else 0); row[3].set_value(value[2] is not None)
        for key,field in self.fields.items(): field.set_value(self.nav.scheduler.config[key])
        self.repeat.model.set_value(self.nav.scheduler.config['repeat_enabled'])
        self.clock.model.get_item_value_model().set_value(0 if self.nav.scheduler.config['clock_mode']=='simulation' else 1)
        self.speed.model.set_value(self.nav.config['local']['max_speed_m_s'])
    def apply_schedule(self):
        from navigation.patrol_scheduler import validate_schedule
        if self.nav.scheduler.state not in ('IDLE','STOPPED','CYCLE_COMPLETE'): raise ValueError('STOP or wait at A before editing schedule')
        config={key:field.as_float for key,field in self.fields.items()}
        if config['max_cycles']!=int(config['max_cycles']): raise ValueError('Max cycles must be integer')
        config['max_cycles']=int(config['max_cycles']); config['repeat_enabled']=self.repeat.model.as_bool
        config['clock_mode']='simulation' if self.clock.model.get_item_value_model().as_int==0 else 'wall'
        validate_schedule(config)
        speed=self.speed.model.as_float
        limit=getattr(self.nav,'speed_limit',.35)
        if math.isfinite(speed) and .12-1e-6<=speed<=limit+1e-6:
            speed=max(.12,min(limit,speed)) # FloatField stores float32 (e.g. 0.300000012).
        if not math.isfinite(speed) or not .12<=speed<=limit: raise ValueError(f'Speed must be 0.12 to {limit:g} m/s')
        self.nav.scheduler.configure(config,self.nav.now); self.nav.config['local']['max_speed_m_s']=speed
        self.nav.before_tick(self.runtime.timeline.is_playing())
        self.nav.reason='Schedule/speed applied; wait measured from completed return to A'
    def start(self):
        self.apply_schedule(); self.nav.start(); self.mode.model.get_item_value_model().set_value(1)
    def save(self): self.apply_schedule(); path=self.nav.save(); self.nav.reason='Saved '+str(path)
    def load(self): self.nav.load(); self.sync(); self.draw()
    def toggle_map(self):
        self.visible_map=not self.visible_map
        if self.visible_map and not self.nav.path and self.nav.b is not None and self.nav.mode=='MANUAL':
            self.call(self.nav.validate_points)
        self.draw()
    def top_view(self):
        from pxr import UsdGeom,Gf
        stage=self.runtime.stage; camera=UsdGeom.Camera.Define(stage,'/World/Navigation/TopCamera')
        xf=UsdGeom.Xformable(camera); xf.ClearXformOpOrder()
        cfg=self.runtime.factory_cfg.get('top_view',{})
        eye=Gf.Vec3d(0,-3 if cfg.get('projection')=='perspective' else 0,cfg.get('height_m',12))
        up=Gf.Vec3d(0,0,1) if cfg.get('projection')=='perspective' else Gf.Vec3d(0,1,0)
        xf.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(eye,Gf.Vec3d(0,0,0),up).GetInverse())
        camera.CreateProjectionAttr(cfg.get('projection','orthographic')); camera.CreateHorizontalApertureAttr(cfg.get('horizontal_aperture',165)); camera.CreateVerticalApertureAttr(cfg.get('vertical_aperture',110))
        if cfg.get('projection')=='perspective': camera.CreateFocalLengthAttr(cfg.get('focal_length_mm',19))
        camera.CreateClippingRangeAttr(Gf.Vec2f(*cfg.get('clipping_m',[9.7,20])))
        self.runtime.set_camera_mode('free')
        if cfg.get('projection')=='perspective':
            # Reuse the initialized render camera; the Kit RTX viewport can
            # render newly bound session cameras black on this installation.
            main=self.runtime.context['camera']
            self.runtime._set_camera_transform(xf.GetLocalTransformation())
            main.CreateProjectionAttr('perspective'); main.CreateFocalLengthAttr(cfg.get('focal_length_mm',19))
            main.CreateHorizontalApertureAttr(cfg['horizontal_aperture']); main.CreateVerticalApertureAttr(cfg['vertical_aperture'])
            main.CreateClippingRangeAttr(Gf.Vec2f(*cfg['clipping_m']))
            self.runtime.context['viewport'].camera_path=str(main.GetPath())
        else: self.runtime.context['viewport'].camera_path=str(camera.GetPath())
    def draw(self):
        from pxr import UsdGeom,Gf
        stage=self.runtime.stage; root=UsdGeom.Xform.Define(stage,'/World/Navigation/Overlay')
        UsdGeom.Imageable(root).CreateVisibilityAttr().Set('inherited' if self.visible_map else 'invisible')
        def lines(name,segments,color):
            if not segments:
                stage.RemovePrim('/World/Navigation/Overlay/'+name)
                return
            curve=UsdGeom.BasisCurves.Define(stage,'/World/Navigation/Overlay/'+name)
            curve.CreateTypeAttr('linear'); curve.CreateCurveVertexCountsAttr([len(s) for s in segments])
            curve.CreatePointsAttr([Gf.Vec3f(float(p[0]),float(p[1]),.025) for s in segments for p in s])
            curve.CreateWidthsAttr([.018]); curve.SetWidthsInterpolation('constant'); curve.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        for key,polys,color in [('AllowedLane',[p['polygon'] for p in self.nav.map_config['allowed_lanes']],(0,1,.15)),('ForbiddenZone',[p['polygon'] for p in self.nav.map_config.get('forbidden_zones',[])],(1,.05,.05))]:
            lines(key,[p+[p[0]] for p in polys],color)
        outlines=[]
        for o in self.nav.detector.obstacles:
            a,b=o['bounds']; outlines.append([a,[b[0],a[1]],b,[a[0],b[1]],a])
        lines('StaticObstacle',outlines,(1,.45,.02))
        dashed=[]
        for a,b in zip(self.nav.path,self.nav.path[1:]):
            length=math.dist(a,b)
            for i in range(math.ceil(length/.25)):
                t=i*.25/max(length,1e-9); u=min(1,(i*.25+.14)/max(length,1e-9))
                dashed.append([[a[j]+(b[j]-a[j])*t for j in [0,1]],[a[j]+(b[j]-a[j])*u for j in [0,1]]])
        lines('PlannedPath',[] if hasattr(self.nav,'route_manager') else dashed,(.1,.6,1))
        if hasattr(self.nav,'route_manager'):
            manager=self.nav.route_manager; rid=self.nav.active_route or self.nav.selected_route()
            route=manager.data['routes'][rid]; ids=route['ordered_nodes']; common=manager.data['common_nodes']; nodes=manager.data['nodes']
            lines('RouteCommonRed',[[nodes[k] for k in common]],(1,.08,.04))
            branch=[nodes[k] for k in ids[len(common)-1:]]
            lines('RouteBranch',[branch],(.05,.3,1) if rid=='route_1_blue' else (.46,.22,.07))
            target=self.nav.edge['goal'] if self.nav.edge else nodes['ENTRY']
            lines('CurrentTarget',[[[target[0]-.2,target[1]],[target[0]+.2,target[1]]],[[target[0],target[1]-.2],[target[0],target[1]+.2]]],(1,.2,1))
        x,y,yaw=pose(self.runtime.last_state)
        radius=self.nav.grid.map.radius
        circle=[[x+radius*math.cos(i*math.pi/16),y+radius*math.sin(i*math.pi/16)] for i in range(33)]
        lines('RobotHeading',[circle,[[x,y],[x+.8*math.cos(yaw),y+.8*math.sin(yaw)]]],(.2,.5,1))
    def update(self):
        now=time.perf_counter()
        if now-self.last_update<.3: return
        self.last_update=now; s=self.nav.snapshot()
        gpu=s['gpu_memory'].get('gpu_device_total_used_mib') if s['gpu_memory'] else None
        clearance='none' if s['nearest_obstacle_m'] is None else f'{s["nearest_obstacle_m"]:.2f}'
        wait=s['next_cycle_seconds']
        wait_text='-' if wait is None else f'{wait:.1f} s ({s["clock_mode"]})'
        if wait is not None and s['clock_mode']=='simulation' and s['loop_fps']>0:
            wait_text+=f' ~{wait*self.runtime.loco_cfg["control_hz"]/s["loop_fps"]:.0f} wall s'
        if s['state']=='CYCLE_COMPLETE': wait_text=('Waiting at ENTRY: ' if hasattr(self.nav,'route_manager') else 'Waiting at A: ')+wait_text
        self.monitor.text=(f'{s["mode"]} | {s["state"]}\nGoal {s["goal"]} | Distance {s["distance_to_goal"]}\n'
            f'XY {s["position"][0]:.2f}, {s["position"][1]:.2f} | speed {math.hypot(*s["actual_velocity"][:2]):.2f}\n'
            f'Completed {s["completed_cycles"]}/{self.nav.scheduler.config["max_cycles"] or "unlimited"} | Repeat {self.nav.scheduler.config["repeat_enabled"]}\n'
            f'{wait_text}\n'
            f'Waypoint {s["active_waypoint"]} | path {s["path_length_m"]:.2f}m\n'
            f'Humans: {s["human_count"]} | within 1.5m: {s["human_within_1_5m"]} (scene truth)\n'
            f'{s["reason"]}\nClearance {clearance} m | deviation {s["path_deviation_m"]:.2f} m\n'
            f'FPS {s["loop_fps"]:.1f} | planner {s["planner_latency_ms"]:.2f} ms | VRAM {gpu} MiB')
        if hasattr(self.nav,'route_manager'):
            status=s['route_validation']; self.monitor.text+=f'\nRoute {s["selected_route"]} ({s["route_mode"]})\nSegment {s["current_segment"]}\nWaypoint {s["current_waypoint"]}\nValidation {status["status"]} | length {status.get("length_m")} m\nWaypoints {status.get("waypoints")} | ETA estimate {status.get("estimated_travel_seconds")} sim s\nNext cycle route {s["next_route"]}'
        self.draw()
    def close(self): self.window.destroy()

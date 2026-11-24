"""Costmap trajectory sampling for the Factory V2 patrol controller."""
import math
import time

from navigation.lane_map import box_distance, segment_distance


class DynamicLocalPlanner:
    max_sim_age = .60
    max_wall_age = 2.0
    max_observed_floor_distance_m = .50

    def __init__(self, safety):
        self.safety = safety
        self.telemetry = {}

    def observation(self, result, epoch, sim_time, wall_time):
        if not result or result.get('perception_epoch') != epoch:
            return None
        grid = result.get('_costmap')
        if grid is None or result.get('points',{}).get('valid',0) == 0:
            return None
        sim_age = sim_time - grid['sim_time']
        wall_age = wall_time - result.get('wall_time_capture', -math.inf)
        if not (0 <= sim_age <= self.max_sim_age and 0 <= wall_age <= self.max_wall_age):
            return None
        return grid

    def trace(self, grid, corridor, pose, command, checkpoint, tolerance, actual=None):
        """Sweep the commanded twist and its braking extension in world XY."""
        self.trace_blocker = None
        x, y, yaw = pose
        vx, vy, rate = command
        if actual is not None:
            vx, vy = actual
            yaw = rate = 0.
        speed = math.hypot(vx, vy)
        horizon = self.safety.config['prediction_seconds']
        duration = horizon + self.safety.stop_distance(speed)/speed if speed else 0.
        if speed and self.safety.simulation_braking_bypass:
            duration = horizon = min(duration,self.max_observed_floor_distance_m/speed)
        resolution = grid['resolution_m']
        count = max(1, math.ceil(speed * duration / (resolution / 3)))
        ox, oy = grid['origin_world_m']
        min_distance = math.inf
        start = x, y
        edge_length = math.dist(corridor.a, checkpoint)
        ux, uy = ((checkpoint[i] - corridor.a[i])/edge_length for i in (0, 1))
        start_along = (x-checkpoint[0])*ux + (y-checkpoint[1])*uy
        start_cross = abs((x-checkpoint[0])*uy-(y-checkpoint[1])*ux)
        for step in range(count + 1):
            t = duration * step / count
            # Integrate a constant twist, then brake along its final tangent.
            turning = min(t, horizon)
            end_yaw = yaw + rate*turning
            if abs(rate) > 1e-6:
                sine = (math.sin(end_yaw) - math.sin(yaw))/rate
                cosine = (math.cos(yaw) - math.cos(end_yaw))/rate
            else:
                sine, cosine = turning*math.cos(yaw), turning*math.sin(yaw)
            straight = t - turning
            point = (x + vx*sine - vy*cosine + straight*(vx*math.cos(end_yaw)-vy*math.sin(end_yaw)),
                     y + vx*cosine + vy*sine + straight*(vx*math.sin(end_yaw)+vy*math.cos(end_yaw)))
            dx, dy = point[0] - checkpoint[0], point[1] - checkpoint[1]
            # Crossing a checkpoint plane off-centre would bypass the checkpoint.
            if speed and dx*ux + dy*uy >= -resolution/2 and abs(dx*uy-dy*ux) > tolerance:
                # Permit lateral recovery if an actual stride already overshot the plane.
                if step and (start_along < -resolution/2 or abs(dx*uy-dy*ux) >= start_cross - 1e-6):
                    self.trace_blocker = 'checkpoint'
                    return None
            if not corridor.contains(point) or not corridor.map.safe(point):
                self.trace_blocker = 'lane_or_footprint'
                return None
            ix, iy = math.floor((point[0]-ox)/resolution), math.floor((point[1]-oy)/resolution)
            if not (0 <= ix < grid['size_cells'] and 0 <= iy < grid['size_cells']):
                self.trace_blocker = 'outside_costmap'
                return None
            clearance = float(grid['distance_m'][ix, iy])
            min_distance = min(min_distance, clearance)
            # The occupied robot cell may be unseen by its own camera. Every
            # new cell must be observed free and outside the inflated footprint.
            if grid['cost'][ix, iy] not in (0, 255):
                self.trace_blocker = 'occupied_or_inflated'
                return None
            if math.dist(start, point) > 1e-8:
                visible_age = grid['free_age_s'][ix, iy]
                same_cell = (ix, iy) == (math.floor((x-ox)/resolution), math.floor((y-oy)/resolution))
                needs_observed_floor = (not self.safety.simulation_braking_bypass and
                                        (actual is not None or speed*t <= self.max_observed_floor_distance_m))
                if needs_observed_floor and not same_cell and not (grid['traversable'][ix, iy] and 0 <= visible_age <= self.max_sim_age):
                    self.trace_blocker = 'floor_unseen_or_stale'
                    return None
        return point, min_distance

    def choose(self, grid, corridor, pose, desired, low, high, cap, checkpoint, tolerance, people, allow_detour=True,
               command_blocker=None):
        began = time.perf_counter()
        self.telemetry = {'status': 'blocked'}
        human_distance = min((box_distance(pose[:2], o['bounds']) - corridor.map.radius
                              for o in people if o['kind'] == 'human'), default=math.inf)
        ox, oy = grid['origin_world_m']
        res = grid['resolution_m']
        for ix, iy in grid.get('human_cells', ()):
            lo = ox+ix*res, oy+iy*res
            hi = lo[0]+res, lo[1]+res
            human_distance = min(human_distance, box_distance(pose[:2], (lo, hi))-corridor.map.radius)
        if self.safety.simulation_braking_bypass:
            human_distance = math.inf
        if human_distance < 1.2:
            cap *= max(0., (human_distance - .65)/.55)
        speed = math.hypot(*desired[:2])
        scale = min(1., cap/speed) if speed else 1.
        nominal = [desired[0]*scale, desired[1]*scale, desired[2]]
        if human_distance <= .65 or (speed and cap < .10):
            self.telemetry['status'] = 'human_wait' if human_distance < 1.2 else 'blocked'
            return None

        def evaluate(command):
            if not all(low[i] <= v <= high[i] for i, v in enumerate(command)):
                self.trace_blocker = 'policy_domain'
                return None
            outcome = self.trace(grid, corridor, pose, command, checkpoint, tolerance)
            if outcome is not None and command_blocker is not None:
                self.trace_blocker = command_blocker(command)
                if self.trace_blocker:
                    return None
            return outcome

        chosen, result = nominal, evaluate(nominal)
        nominal_blocker = self.trace_blocker
        if result is None and speed:
            direction = math.atan2(nominal[1], nominal[0])
            best = -math.inf
            # Keep heading control separate from side-stepping. Sample slower
            # twists too, so the sweep shrinks as the obstacle gets closer.
            angles = (0, 25, -25, 45, -45, 70, -70, 90, -90) if allow_detour and human_distance >= 1.2 else (0,)
            nominal_speed = math.hypot(*nominal[:2])
            speeds = {nominal_speed*ratio for ratio in (1., .75, .5, .35)}
            # A high speed selection must still offer usable crawl commands
            # when the forward camera has only observed a short stopping path.
            speeds.update((.10,.12,.15,.18,.22))
            for candidate_speed in sorted(speeds, reverse=True):
                if not .10 <= candidate_speed <= nominal_speed:
                    continue
                for degrees in angles:
                    angle = direction + math.radians(degrees)
                    candidate = [candidate_speed*math.cos(angle), candidate_speed*math.sin(angle), nominal[2]]
                    outcome = evaluate(candidate)
                    if outcome is None:
                        continue
                    end, clearance = outcome
                    progress = math.dist(pose[:2], checkpoint) - math.dist(end, checkpoint)
                    deviation = segment_distance(end, corridor.a, checkpoint)
                    score = progress - .35*deviation + .12*min(clearance, 1.) - .001*abs(degrees)
                    if score > best:
                        best, chosen, result = score, candidate, outcome
        if result is not None:
            end, clearance = result
            self.telemetry.update(status='tracking' if chosen == desired else 'adjusted',
                                  predicted_end_xy=end, predicted_obstacle_cell_distance_m=clearance if math.isfinite(clearance) else None)
        self.telemetry['nominal_blocker'] = nominal_blocker
        self.telemetry['planner_ms'] = (time.perf_counter()-began)*1000
        return chosen if result is not None else None

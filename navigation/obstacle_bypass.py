"""Side-step bypass around an obstacle that blocks a straight mandatory route edge.

Pure geometry: given the edge a->b, the robot position and the scene
obstacles, return world waypoints that leave the centreline, pass the nearest
blocking obstacle at a constant lateral offset and rejoin the centreline
before the edge's checkpoint. Every waypoint and every connecting segment is
checked against the full-footprint lane map (lanes, red forbidden zones and
all obstacles). People get extra clearance, matching the costmap inflation.
"""
import math

HUMAN_EXTRA_M = .40      # costmap human_extra_m (0.3) + one 0.1 m costmap cell
OBJECT_EXTRA_M = .10     # one costmap cell
PASS_MARGIN_M = .05
RAMP_MIN_M = .60
OFFSET_STEP_M = .05
OFFSET_MAX_M = 2.2
SAMPLE_M = .05


def expanded(o):
    extra = HUMAN_EXTRA_M if o.get('kind') == 'human' else OBJECT_EXTRA_M
    lo, hi = o['bounds']
    return [[lo[0]-extra, lo[1]-extra], [hi[0]+extra, hi[1]+extra]]


class Frame:
    def __init__(self, a, b):
        self.a = a; self.length = math.dist(a, b)
        self.u = ((b[0]-a[0])/self.length, (b[1]-a[1])/self.length)
        self.n = (-self.u[1], self.u[0])

    def local(self, p):
        dx, dy = p[0]-self.a[0], p[1]-self.a[1]
        return dx*self.u[0]+dy*self.u[1], dx*self.n[0]+dy*self.n[1]

    def world(self, s, d):
        return (self.a[0]+s*self.u[0]+d*self.n[0], self.a[1]+s*self.u[1]+d*self.n[1])

    def extent(self, bounds):
        lo, hi = bounds
        corners = [self.local(c) for c in ((lo[0], lo[1]), (hi[0], lo[1]), (hi[0], hi[1]), (lo[0], hi[1]))]
        return (min(c[0] for c in corners), max(c[0] for c in corners),
                min(c[1] for c in corners), max(c[1] for c in corners))


def blocking(frame, robot_s, obstacles, radius):
    """Obstacles ahead whose (expanded) footprint the centreline sweep would hit."""
    found = []
    for o in obstacles:
        s0, s1, c0, c1 = frame.extent(expanded(o))
        if c0 < radius and c1 > -radius and s1 > robot_s - radius and s0 < frame.length + radius:
            found.append((s0, s1, o))
    return sorted(found, key=lambda item: item[0])


def segment_safe(safe, p, q):
    count = max(1, math.ceil(math.dist(p, q)/SAMPLE_M))
    return all(safe((p[0]+(q[0]-p[0])*i/count, p[1]+(q[1]-p[1])*i/count)) for i in range(count+1))


def _gate_ok(offset, gates, enter_s, exit_s, gate_half_width):
    """A pass-through checkpoint inside the offset stretch must be crossed within its gate."""
    if gate_half_width is None:
        return not any(enter_s-1e-6 <= g <= exit_s+1e-6 for g in gates)
    return abs(offset) <= gate_half_width+1e-9 or not any(enter_s-1e-6 <= g <= exit_s+1e-6 for g in gates)


def plan(a, b, robot_xy, obstacles, lane_map_factory, radius, ramp_slope=None, gates=(), gate_half_width=None):
    """Return {'obstacle','points','path','offset_m','start_s'} or None.

    lane_map_factory(obstacles) -> object with safe(point) using the full
    robot footprint. `obstacles` have 'bounds'/'kind' (scene colliders or
    costmap blobs). b is the next node where the robot must stop or turn;
    `gates` are along-edge positions of pass-through checkpoints between a
    and b, which the bypass may cross off-centre by up to gate_half_width.
    ramp_slope (lateral/forward) makes the lane change an early diagonal
    instead of the steep RAMP_MIN_M side-step.
    """
    frame = Frame(a, b)
    robot_s, _ = frame.local(robot_xy)
    ahead = blocking(frame, robot_s, obstacles, radius)
    if not ahead:
        return None
    s0, s1, target = ahead[0]
    # Neighbours that lie along the same passage are bypassed together.
    group = [target]
    for t0, t1, other in ahead[1:]:
        if t0 <= s1 + 2*radius + RAMP_MIN_M:
            group.append(other); s1 = max(s1, t1)
    ids = {id(o) for o in group}
    inflated = [dict(o, bounds=expanded(o)) if id(o) in ids else o for o in obstacles]
    safe = lane_map_factory(inflated).safe
    pass_start = s0 - radius - PASS_MARGIN_M
    pass_end = s1 + radius + PASS_MARGIN_M
    if pass_end >= frame.length - .05:
        return None     # the stop/turn node itself is blocked: wait instead of skipping it
    options = []
    for side in (1, -1):
        feasible = []
        d = OFFSET_STEP_M
        while d <= OFFSET_MAX_M + 1e-9:
            offset = side*d
            if all(safe(frame.world(pass_start+i*SAMPLE_M, offset))
                   for i in range(int((pass_end-pass_start)/SAMPLE_M)+1)):
                feasible.append(d)
            elif feasible:
                options.append((feasible[-1]-feasible[0], -feasible[0], side, feasible[0], feasible[-1]))
                feasible = []
            d += OFFSET_STEP_M
        if feasible:
            near, far = feasible[0], feasible[-1]
            options.append((far-near, -near, side, near, far))
    ranked = sorted(options, reverse=True)
    # A shelf at the rejoin point can require staying alongside the lane
    # longer. Keep the normal rejoin first and never pass the next turn.
    for extension in (0., .6, 1.2, 1.8, 2.4):
        rejoin_start = pass_end + extension
        if rejoin_start >= frame.length-.05:
            break
        result = _choose_path(ranked, frame, robot_xy, robot_s, safe, pass_start, rejoin_start,
                              group, gates, gate_half_width, ramp_slope)
        if result is not None:
            return result
    return None


def _choose_path(options, frame, robot_xy, robot_s, safe, pass_start, pass_end, group,
                 gates, gate_half_width, ramp_slope):
    for _, _, side, near, far in options:
        preferred = near+min(.20, (far-near)/2)
        offsets = [side*preferred]
        if gate_half_width is not None and near <= gate_half_width < preferred:
            offsets.append(side*gate_half_width)
        # A clear parallel stretch does not guarantee that its entry and
        # exit ramps fit. Try other clear offsets before declaring a dead end.
        samples = [near+i*OFFSET_STEP_M for i in range(round((far-near)/OFFSET_STEP_M)+1)]
        offsets.extend(side*d for d in sorted(samples,key=lambda d:abs(d-preferred)))
        for offset in dict.fromkeys(offsets):
            ramps = [max(RAMP_MIN_M, abs(offset))]
            if ramp_slope:
                ramps.insert(0, max(RAMP_MIN_M, abs(offset)/ramp_slope))
            for ramp in dict.fromkeys(ramps):
                result = _build(frame, robot_xy, robot_s, safe, offset, ramp, pass_start, pass_end, group,
                                gates, gate_half_width)
                if result is not None:
                    return result
    return None


def _build(frame, robot_xy, robot_s, safe, offset, ramp, pass_start, pass_end, group, gates, gate_half_width):
    exit_s = min(frame.length, pass_end+ramp)
    if robot_s >= pass_start-ramp:
        # Already at/inside the bypass: start from the real pose and do
        # not walk back to a pass point that is behind the robot.
        enter_s, entry = robot_s, tuple(robot_xy)
        first = frame.world(max(pass_start, robot_s), offset)
    else:
        enter_s = pass_start-ramp
        entry, first = frame.world(enter_s, 0.), frame.world(pass_start, offset)
    if not _gate_ok(offset, gates, enter_s, exit_s, gate_half_width):
        return None
    points = [first, frame.world(pass_end, offset), frame.world(exit_s, 0.)]
    path = [entry]+points
    if not all(segment_safe(safe, p, q) for p, q in zip(path, path[1:])):
        return None
    return {'obstacle': [o.get('id') for o in group], 'points': points, 'path': path,
            'offset_m': offset, 'start_s': enter_s, 'pass_s': [pass_start, pass_end], 'ramp_m': ramp}


def path_clear(robot_xy, points, obstacles, lane_map_factory, skip_m=.10):
    """True while the remaining bypass polyline still clears the current obstacles.

    Used to keep an active bypass stable while costmap blobs jitter: replan
    only when the path actually becomes unsafe. People keep their extra
    clearance; other obstacles use the plain robot footprint.
    """
    if not points:
        return False
    inflated = [dict(o, bounds=expanded(o)) if o.get('kind') == 'human' else o for o in obstacles]
    safe = lane_map_factory(inflated).safe
    path = [tuple(robot_xy)]+[tuple(p) for p in points]
    for p, q in zip(path, path[1:]):
        count = max(1, math.ceil(math.dist(p, q)/SAMPLE_M))
        for i in range(count+1):
            point = (p[0]+(q[0]-p[0])*i/count, p[1]+(q[1]-p[1])*i/count)
            if math.dist(point, robot_xy) >= skip_m and not safe(point):
                return False
    return True

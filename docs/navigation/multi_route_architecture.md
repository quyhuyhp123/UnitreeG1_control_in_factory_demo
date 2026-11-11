# Multi-route architecture

V2 selects `MultiRouteController`, a subclass of the legacy A/B controller. V1,
or V2 with `multi_route_enabled=false`, continues to use the original A/B path.
The trained policy, factory geometry, robot model and raw safety envelope are unchanged.

## Source mapping

`references/patrol_routes_reference.png` was opened and inspected. The red route
goes through the left and north corridors; blue returns down the centre, brown
returns down the right. World coordinates are derived from the real USD/map
outer boundary, island holes and verified entry marker, not image pixels.
The full node table is in `configs/patrol_routes.json`.

| Node | World XY metres |
|---|---|
| ENTRY / EXIT | -8.85, -13.75 |
| ENTRY_INNER | -6.985646, -13.0 |
| LEFT_BOTTOM | -6.985646, -10.1 |
| LEFT_CORRIDOR | -6.985646, 0 |
| TOP_LEFT_TURN | -6.985646, 10.952830 |
| TOP_CENTER_JUNCTION | 0, 10.952830 |
| CENTER_CORRIDOR | 0, 0 |
| BOTTOM_CENTER_JUNCTION | 0, -11.4 |
| TOP_RIGHT_TURN | 6.95, 10.952830 |
| RIGHT_CORRIDOR | 6.95, 0 |
| BOTTOM_RIGHT_TURN | 6.95, -11.4 |
| BOTTOM_EXIT_CORRIDOR | -6.985646, -11.4 |

Left, centre and north coordinates are corridor midpoints. Right X=6.95 is the
verified safe grid centre nearest the right corridor midpoint. Bottom Y=-11.4
lies below both island ends and above the south wall; the entry elbow avoids
the lower-left boundary notch. EXIT stays at the verified **inner** entry marker:
there is no external floor/allowed polygon to support walking fully outside.

## Graph and planning

Route 1: ENTRY → ENTRY_INNER → LEFT_BOTTOM → LEFT_CORRIDOR → TOP_LEFT_TURN →
TOP_CENTER_JUNCTION → CENTER_CORRIDOR → BOTTOM_CENTER_JUNCTION →
BOTTOM_EXIT_CORRIDOR → ENTRY_INNER → EXIT.

Route 2 uses the same common prefix, then TOP_RIGHT_TURN → RIGHT_CORRIDOR →
BOTTOM_RIGHT_TURN → BOTTOM_CENTER_JUNCTION → BOTTOM_EXIT_CORRIDOR → ENTRY_INNER → EXIT.

Geometry validation lengths: about 65.90 m and 79.80 m. Lengths/edge paths are
recorded in `multi_route_geometry.json`; VALID geometry is not physical acceptance.

`CorridorGrid` restricts the existing A* to a 0.40 m centreline capsule for each
mandatory edge. Physical footprint remains radius 0.67 m, including all original
body/sway/pose/safety margins. Obstacles and forbidden islands remain active.
Each edge is separately planned/checked; no smoothing across mandatory nodes,
straight-line fallback, skipped checkpoint or automatic cross-route fallback.

The actual measured pose may be a virtual start node when the full footprint
fits but a conservative whole-cell mask rejects its cell. It is not replaced
by that cell centre. Dense full-radius checks validate each connector. Normal
A* nodes retain whole-cell clearance and diagonal-corner protection. Replanning
uses a short recovery anchor on the original centreline, not a long diagonal
that leaves cross-track error uncorrected for tens of metres.

## Motion and safety

Each edge is densified at 0.20 m before LocalPlanner lookahead. Collinear nodes
can advance within tolerance without a full stop. A large corner is completed
by positional capture, next-edge yaw alignment and measured speed settling.
Commands become bounded joint targets only through the existing trained policy.

Multi-route acceleration couples linear XY components to preserve vector
direction. Independent component ramps could turn a safe almost-forward command
toward a nearby wall at the first ramp step. The old A/B ramp stays the default.

Prediction uses a full-stride 30-sample velocity mean. Mean speed below 0.03 m/s
is standing sway; raw pose/tilt/speed/human checks still run. Reaction latency
increases to 0.65 s for filtering delay. No footprint or braking margin is reduced.
The existing SafetySupervisor is reused. Replanning stays on the same edge,
retains leg deadlines and waits/faults if no constrained path exists.

RouteScheduler reuses clock/limits/pause logic. Only verified EXIT settling
increments a cycle. Alternate is chosen from completed cycles; STOP/FAULT never
counts completion. Defaults are one cycle, repeat off, 5 s start delay, 120 s
after-return interval. Physical tests may shorten waits and must report that.

Configuration saves include factory identity, source hash and full route graph.
Loading requires STOP/Manual and revalidates routes. No camera AI, model training,
robot network connection, geometry changes or policy replacement is involved.

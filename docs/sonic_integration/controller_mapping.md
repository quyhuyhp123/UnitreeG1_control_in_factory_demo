# Mapping audit
Isaac feedback uses world xyz, normalized WXYZ quaternion, world angular velocity and joint names. Planner uses world Z-up, X-forward/Y-left, qpos [xyz, WXYZ, 29 tree-order angles]. No WXYZ/XYZW conversion is needed for planner qpos. Explicit conversion utilities exist for interfaces that require XYZW; unit tests cover normalization, roundtrip and body rotation.

Official policy_parameters.hpp lists mujoco_to_isaaclab = [0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28]. sonic_joint_adapter stores this as POLICY_FROM_SDK and computes inverse. Never use live PhysX index order as policy order; names must be matched explicitly. No SONIC action scaling, gains or target writes are implemented.

Planner actual I/O: context float32 [1,4,36]; target_vel float32 [1]; mode int64 [1]; movement/facing float32 [1,3]; height float32 [1]; random_seed int64 [1]; has_specific_target int64 [1,1]; specific_target_positions float32 [1,4,3]; specific_target_headings float32 [1,4]; allowed_pred_num_tokens int64 [1,11]. Output qpos float32 [1,64,36], num_pred_frames int32 [1] (actual model differs from documentation's int64 description). Slice valid frame count.

Advanced defaults are copied from pinned InitializeSpecific: no waypoint override, zero inactive positions/headings, token mask [1,1,1,1,1,1,0,0,0,0,0], height -1; seed 1234 from planner config. Idle inference uses four consecutive actual PhysX telemetry states, not random/standing placeholders. Full temporal alignment remains unvalidated for online use.

Official motion IDs: idle 0, slowWalk 1, walk 2, run 3. Offline adapter maps body horizontal velocity to world movement and integrates requested yaw for facing. Target speed <=0 means use mode default, so a stopped command explicitly selects idle. Running and non-idle commands have not been validated and are not exposed for active SONIC control.

Source: [policy constants](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/1e1ec4f5f27e6c3e03048f693bd7c60cce04afda/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp), [planner specification](https://nvlabs.github.io/GR00T-WholeBodyControl/references/planner_onnx.html).

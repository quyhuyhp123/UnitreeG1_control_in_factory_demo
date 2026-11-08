# Compatibility: PARTIALLY IMPLEMENTED
Inspected 2026-11-08. Source commit 1e1ec4f5f27e6c3e03048f693bd7c60cce04afda; model revision 6733128a3d8a523b1418b06bca3cdf61c8b0987f.

| Component | Status | Evidence / limitation |
|---|---|---|
| Existing Locomotion RL + Windows PhysX | Verified compatible for tested scene | Fresh flat and factory regression passed |
| Default SONIC encoder/decoder ONNX loading | Partially compatible | ORT 1.26 CPU session creation passed; no validated full-input inference |
| Default SONIC planner CPU ONNX | Verified compatible for offline idle case only | Three valid real-state inference runs, not closed-loop control |
| Official C++/TensorRT deployment on native Windows | Not compatible with documented deployment workflow | Upstream requires Debian-based Linux and desktop TensorRT 10.13; no native build attempted |
| Isaac Lab PyTorch evaluation locally | Not compatible with current installed environment | Isaac Python has no torch or isaaclab; no installation attempted |
| SONIC source on Isaac Lab 3.0 EA / Sim 6.1 | Not tested | SONIC source advertises Lab 2.3.2 / Python 3.11 workflow; cannot infer 3.0 compatibility |
| Low latency / v1.1 | Not tested | Listed in official release manifest; weights not downloaded |
| SONIC control of current G1 in PhysX | Not tested / BLOCKED | Full adapter and robot parity unresolved; selection rejects before app startup |

Local verified environment: Windows 11 Pro 10.0.26200, RTX 5070 12227 MiB, driver 610.88; Isaac C:/isaacsim version 6.1.0-rc.26+release.49347.2d230af4.gl, bundled Python 3.12.13. Blender D:/Blender/blender.exe is not involved. Existing ONNX extension at C:/isaacsim/exts/isaacsim.pip.onnx/pip_prebundle supplies ORT. ORT advertises CUDA/TensorRT providers, but availability is not execution validation; only CPU provider was used. Direct Isaac-launcher imports found torch, isaaclab, tensorrt and huggingface_hub absent. ORT becomes available through its existing extension, not a pip install.

Concrete remaining work:
1. Port and compare NVIDIA motion-data FK/body quaternion generation, heading alignment, reference velocity and trajectory/history management. Current planner output is qpos, while encoder gatherers consume motion reference including body rotations. No placeholders were fed to the encoder.
2. Establish exact enabled encoder layout (1762), decoder 10-frame history (994), reset padding, relative joint positions and previous-action conventions against official gatherers. YAML header's old 436 comment is not the model contract.
3. Resolve robot model/actuator parity: NVIDIA policy_parameters.hpp uses N7520-22 for hip pitch, different nominal pose (-0.312, knee 0.669, ankle -0.363) and motor-derived action scales/gains. Current config assigns hip pitch N7520-14.3 and uses the verified Mjlab deploy profile. Joint-name permutation alone does not prove equivalent mass, inertia, collision or actuator dynamics. Existing gains/assets were preserved.
4. Validate encoder+decoder real-reference inference, then isolated flat PhysX, then factory. Do not enable runtime SONIC before those pass.

These are unresolved porting/validation tasks, not proof that a native Windows ONNX port is impossible. Linux inference is an alternative to evaluate later; no WSL, remote machine or robot network was configured.

Official references:
- [Pinned SONIC source](https://github.com/NVlabs/GR00T-WholeBodyControl/tree/1e1ec4f5f27e6c3e03048f693bd7c60cce04afda)
- [Deployment prerequisites](https://nvlabs.github.io/GR00T-WholeBodyControl/getting_started/installation_deploy.html)
- [Isaac Lab 3.0 EA installation: Sim 6.1, Python 3.12](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html)
- [Isaac Sim 6.1 requirements: minimum 16 GB VRAM](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/installation/requirements.html)

12 GB is below official minimum. Tested factory peak whole-device VRAM was 4389 MiB, not a SONIC combined-rendering measurement. D3D12 remains selected, no path tracing, one robot/environment, existing RGB head preview retained; no depth/LiDAR/training. No software upgrade or driver change.

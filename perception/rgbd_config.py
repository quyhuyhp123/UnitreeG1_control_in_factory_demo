"""RGB-D camera configuration. Standard library only so `--check` runs without Isaac Sim."""
from pathlib import Path
import json

ROOT=Path(__file__).resolve().parents[1]


def load_config(overrides=None):
    config=json.loads((ROOT/'configs/rgbd_camera.json').read_text())
    config.update(overrides or {})
    stabilization=config.get('stabilization',{})
    if stabilization.get('enabled',False):
        if any(not .01<=float(stabilization[k])<=2 for k in ['position_tau_s','height_tau_s','yaw_tau_s']):raise ValueError('Invalid camera smoothing time constant')
        if not .01<=float(stabilization['max_position_lag_m'])<=.2:raise ValueError('Invalid virtual camera lag bound')
    width,height=config['resolution']
    if not (16<=width<=1920 and 16<=height<=1080): raise ValueError('Invalid RGB-D resolution')
    if not 10<=config['horizontal_fov_deg']<=150 or not -10<=config['pitch_down_deg']<=80: raise ValueError('Invalid RGB-D optics/pitch')
    near,far=config['clipping_range_m']; lo,hi=config['valid_depth_range_m']
    if not 0<near<=lo<hi<=far: raise ValueError('Depth range must lie inside the clipping range')
    if not 1<=config['capture_hz']<=50 or not 0<=config['pose_lag_ticks']<=5: raise ValueError('Invalid RGB-D capture rate/pose lag')
    return config

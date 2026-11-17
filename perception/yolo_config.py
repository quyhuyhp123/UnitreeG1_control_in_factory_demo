"""YOLO configuration and model check. Standard library only so `--check` runs without Isaac Sim."""
from pathlib import Path
import hashlib,json

ROOT=Path(__file__).resolve().parents[1]


def load_config(overrides=None):
    config=json.loads((ROOT/'configs/yolo.json').read_text())
    config.update(overrides or {})
    if config['imgsz']%32 or not 160<=config['imgsz']<=1280: raise ValueError('YOLO imgsz must be a multiple of 32')
    if not 0<config['confidence_threshold']<1 or not 0<config['iou_threshold']<1: raise ValueError('Invalid YOLO thresholds')
    if not 1<=config['target_hz']<=30: raise ValueError('Invalid YOLO target rate')
    if config['provider'] not in ('cuda','cpu'): raise ValueError('YOLO provider must be cuda or cpu')
    return config


def verify_model(config):
    path=ROOT/config['model']
    if not path.is_file(): raise FileNotFoundError(f'Missing YOLO model: {path} (see scripts/export_yolo_onnx.py)')
    if hashlib.sha256(path.read_bytes()).hexdigest()!=config['model_sha256']: raise ValueError('YOLO model hash mismatch')
    return path

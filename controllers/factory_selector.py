"""Restart-only factory selection; no live scene switching."""
from pathlib import Path
import json
ROOT=Path(r'D:\unitree_g1')
def load_factory(factory=None,config=None):
    selector=json.loads((ROOT/'configs/factory_selector.json').read_text())
    name=factory or selector['default_factory']
    if name not in selector['factories']: raise ValueError('Unknown factory '+name)
    path=Path(config) if config else ROOT/selector['factories'][name]
    cfg=json.loads(path.read_text(encoding='utf-8-sig'))
    if factory and cfg.get('factory_id','v1')!=factory: raise ValueError('Factory/config selection mismatch')
    return cfg

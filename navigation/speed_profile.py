"""Factory V2 speed ceiling, distinct from the policy's command range."""
from pathlib import Path
import json,math
ROOT=Path(r'D:\unitree_g1')

def load_speed_profile():
    profile=json.loads((ROOT/'configs/navigation_speed_v2.json').read_text())
    cfg=json.loads((ROOT/'configs/locomotion_mjlab.yaml').read_text())
    factory=json.loads((ROOT/'configs/factory_v2.json').read_text())
    if profile['version']!=1 or profile['factory_id']!='v2' or profile['checkpoint_sha256']!=cfg['checkpoint_sha256']:
        raise ValueError('Speed profile identity/checkpoint mismatch')
    if profile['source_factory_sha256']!=factory['source_factory_sha256'] or profile['factory_usd']!=factory['factory_usd']:
        raise ValueError('Speed profile factory geometry mismatch')
    maximum=profile['maximum_verified_m_s']; default=profile['default_m_s']; actual=profile['max_actual_speed_m_s']
    if not all(math.isfinite(v) for v in (maximum,default,actual)) or not .12<=default<=maximum<=1. or actual<maximum:
        raise ValueError('Invalid verified speed profile')
    if maximum>.22:
        evidence=(ROOT/profile['evidence']).resolve()
        if not evidence.is_relative_to(ROOT): raise ValueError('Speed evidence must stay inside project')
        result=json.loads(evidence.read_text())
        if result.get('status')!='passed' or abs(result.get('candidate_speed_m_s',0)-maximum)>1e-6:
            raise ValueError('Speed ceiling needs a passing factory route audit')
    return profile

"""Factory V2 speed ceiling, distinct from the policy's command range."""
from pathlib import Path
import json,math
ROOT=Path(r'D:\unitree_g1')

def validate_speed_profile(profile,cfg,factory,root=ROOT):
    if profile['version']!=1 or profile['factory_id']!='v2' or profile['checkpoint_sha256']!=cfg['checkpoint_sha256']:
        raise ValueError('Speed profile identity/checkpoint mismatch')
    if profile['source_factory_sha256']!=factory['source_factory_sha256'] or profile['factory_usd']!=factory['factory_usd']:
        raise ValueError('Speed profile factory geometry mismatch')
    maximum=profile['maximum_verified_m_s']; default=profile['default_m_s']; actual=profile['max_actual_speed_m_s']
    override=profile.get('unverified_user_override',False)
    if type(override) is not bool: raise ValueError('Speed override must be an explicit boolean')
    if override and (not isinstance(profile.get('override_note'),str) or not profile['override_note'].strip()):
        raise ValueError('Speed override needs an explanatory note')
    if not all(math.isfinite(v) for v in (maximum,default,actual)) or not .12<=default<=maximum<=1. or actual<maximum:
        raise ValueError('Invalid verified speed profile')
    if maximum>.22:
        evidence=(root/profile['evidence']).resolve()
        if not evidence.is_relative_to(root.resolve()): raise ValueError('Speed evidence must stay inside project')
        result=json.loads(evidence.read_text(encoding='utf-8-sig'))
        if result.get('status')!='passed' or (not override and abs(result.get('candidate_speed_m_s',0)-maximum)>1e-6):
            raise ValueError('Speed ceiling needs a passing factory route audit')
    return profile

def load_speed_profile():
    return validate_speed_profile(
        json.loads((ROOT/'configs/navigation_speed_v2.json').read_text(encoding='utf-8-sig')),
        json.loads((ROOT/'configs/locomotion_mjlab.yaml').read_text(encoding='utf-8-sig')),
        json.loads((ROOT/'configs/factory_v2.json').read_text(encoding='utf-8-sig')))

"""Factory generator entry point. Uses the preserved base and industrial revision."""
from pathlib import Path
project_root = Path(r'D:\unitree_g1')
script = project_root / 'blender' / 'generate_factory_v2.py'
exec(compile(script.read_text(encoding='utf-8'), str(script), 'exec'))

"""Generate three referenced USD maps; no simulation or training."""
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
if __name__=='__main__':
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'extra_args':['--/app/vulkan=false']})
    try:
        from controllers.factory_maps import build
        for map_id in (1,2,3):print(build(map_id),flush=True)
    except BaseException:
        import traceback
        traceback.print_exc();sys.stderr.flush();raise
    finally:app.close()

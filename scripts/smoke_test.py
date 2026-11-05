"""Empty-stage startup probe. Run with the verified Isaac Sim Python launcher."""
from pathlib import Path
import json
import sys
import traceback

PROJECT_ROOT = Path(r"D:\unitree_g1")
RESULT = PROJECT_ROOT / "logs" / "smoke_result.json"

def main():
    app = None
    result = {"status": "started", "python": sys.executable, "updates": 0}
    RESULT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    try:
        from isaacsim import SimulationApp
        app = SimulationApp({"headless": True, "width": 640, "height": 480})
        import omni.usd
        omni.usd.get_context().new_stage()
        assert omni.usd.get_context().get_stage() is not None
        for _ in range(120):
            app.update()
            result["updates"] += 1
        # Kit fastShutdown may terminate the process inside close(). Persist
        # the completed-stage evidence before requesting shutdown.
        result["status"] = "stage_updates_passed"
    except BaseException:
        result["status"] = "failed"
        result["error"] = traceback.format_exc()
        raise
    finally:
        RESULT.write_text(json.dumps(result, indent=2), encoding="utf-8")
        if app is not None:
            app.close()
    result["status"] = "passed"
    RESULT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print("EMPTY_STAGE_SMOKE_PASS", flush=True)

if __name__ == "__main__":
    main()

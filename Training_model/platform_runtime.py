"""Native worker launch/stop on Windows; preserve existing Linux launcher."""
from pathlib import Path
import os,sys,subprocess,signal
from functools import lru_cache
HERE=Path(__file__).resolve().parent

def python_path():
    configured=os.environ.get('G1_TRAINING_PYTHON')
    candidates=[Path(configured)] if configured else [HERE/'.venv/Scripts/python.exe',HERE.parent/'.venv/yolo/Scripts/python.exe']
    for path in candidates:
        if path.is_file():return path
    raise FileNotFoundError('Set G1_TRAINING_PYTHON to an isolated Windows Python with required ML dependencies')

def ml_command(*arguments):
    if os.name=='nt':return [str(python_path()),'-u',*[str(a) for a in arguments]]
    return ['bash',str(HERE/'ml_python.sh'),*[str(a) for a in arguments]]

def worker_environment():
    env=os.environ.copy()
    if os.name=='nt':
        for key in ['PYTHONHOME','PYTHONPATH','PYTHONEXE']:env.pop(key,None)
    settings=HERE/'.ultralytics';settings.mkdir(exist_ok=True)
    env.update(YOLO_CONFIG_DIR=str(settings),YOLO_AUTOINSTALL='false',PYTHONUNBUFFERED='1',PYTHONUTF8='1',MPLBACKEND='Agg')
    return env

def process_options():
    return {'creationflags':subprocess.CREATE_NO_WINDOW|subprocess.CREATE_NEW_PROCESS_GROUP} if os.name=='nt' else {'start_new_session':True}

def stop_process(process):
    if process is None or process.poll() is not None:return
    if os.name=='nt':
        subprocess.run(['taskkill','/PID',str(process.pid),'/T','/F'],capture_output=True,creationflags=subprocess.CREATE_NO_WINDOW)
    else:
        try:os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:return
    try:process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        if os.name!='nt':
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        else:process.kill()
        process.wait(timeout=5)

def dependencies_ready():
    if os.name!='nt':return (HERE/'.deps/ultralytics').is_dir()
    try:
        result=subprocess.run(ml_command('-c',"import torch,ultralytics,PIL"),env=worker_environment(),capture_output=True,timeout=20,**process_options())
        return result.returncode==0
    except (OSError,subprocess.TimeoutExpired):return False

@lru_cache(maxsize=1)
def cuda_available():
    if os.name!='nt':return True # Existing Linux backend selection is preserved.
    try:
        result=subprocess.run(ml_command('-c','import torch;print(int(torch.cuda.is_available()))'),env=worker_environment(),capture_output=True,timeout=20,**process_options())
        return result.returncode==0 and result.stdout.strip()==b'1'
    except (OSError,subprocess.TimeoutExpired):return False

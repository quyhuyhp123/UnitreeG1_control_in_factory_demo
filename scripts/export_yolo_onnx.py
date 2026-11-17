"""One-off export of pretrained YOLO11 weights to ONNX. Not used at runtime.

The simulation only needs onnxruntime, which Isaac Sim already ships. This
script needs Ultralytics, which is deliberately NOT installed into Isaac Sim.
Install the export tools into a throwaway directory and reuse Isaac's torch:

    IS=$ISAAC_SIM_ROOT; TOOLS=/path/to/empty/dir
    $IS/kit/python/bin/python3 -m pip install --no-deps --target $TOOLS \
        ultralytics ultralytics-thop onnx protobuf ml_dtypes polars polars-runtime-32 tqdm
    export PYTHONPATH=$IS/extsDeprecated/omni.isaac.ml_archive/pip_prebundle:$IS/exts/isaacsim.pip.nv/pip_prebundle:\
$IS/exts/omni.pip.compute/pip_prebundle:$(ls -d $IS/extscache/omni.kit.pip_archive*/pip_prebundle):\
$IS/exts/omni.isaac.core_archive/pip_prebundle:$IS/exts/omni.pip.cloud/pip_prebundle:$TOOLS
    $IS/kit/python/bin/python3 scripts/export_yolo_onnx.py --imgsz 320

`--no-deps` and a separate `--target` keep Isaac Sim's torch, CUDA, numpy and
OpenCV untouched; $TOOLS goes last on PYTHONPATH so Isaac's packages win.
After exporting, update model/model_sha256/imgsz in configs/yolo.json.
"""
from pathlib import Path
import argparse,hashlib,json,os,shutil,tempfile
ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--weights',default='yolo11n.pt'); parser.add_argument('--imgsz',type=int,default=320)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    work=Path(tempfile.mkdtemp(prefix='yolo_export_')); os.environ.setdefault('YOLO_CONFIG_DIR',str(work)); os.chdir(work)
    from ultralytics import YOLO
    model=YOLO(args.weights)
    exported=Path(model.export(format='onnx',imgsz=args.imgsz,opset=17,simplify=False,dynamic=False)).resolve()
    stem=Path(args.weights).stem
    output=args.output or ROOT/'checkpoints/yolo'/(f'{stem}.onnx' if args.imgsz==320 else f'{stem}_{args.imgsz}.onnx')
    output.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(exported,output)
    (output.parent/'coco_names.json').write_text(json.dumps(model.names))
    print(json.dumps({'onnx':str(output),'sha256':hashlib.sha256(output.read_bytes()).hexdigest(),'imgsz':args.imgsz,'classes':len(model.names)}))

if __name__=='__main__': main()

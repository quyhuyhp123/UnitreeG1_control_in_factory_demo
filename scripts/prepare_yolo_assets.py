"""Download official pretrained weights/sample photos; no training or Isaac imports."""
from pathlib import Path
import hashlib, json, urllib.request, zipfile, io
ROOT=Path(r'D:\unitree_g1')

def fetch(url, destination):
    destination.parent.mkdir(parents=True,exist_ok=True)
    if not destination.exists():
        temporary=destination.with_suffix(destination.suffix+'.part')
        urllib.request.urlretrieve(url, temporary)
        temporary.replace(destination)
    return {'path':str(destination.relative_to(ROOT)),'url':url,
            'sha256':hashlib.sha256(destination.read_bytes()).hexdigest(),'bytes':destination.stat().st_size}

def main():
    records=[]
    records.append(fetch('https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt',ROOT/'checkpoints/yolo/yolo11n.pt'))
    records.append(fetch('https://raw.githubusercontent.com/ultralytics/ultralytics/v8.3.221/ultralytics/assets/bus.jpg',ROOT/'logs/yolo_samples/bus.jpg'))
    archive=ROOT/'logs/yolo_samples/coco128.zip'
    records.append(fetch('https://github.com/ultralytics/assets/releases/download/v0.0.0/coco128.zip',archive))
    # Only extract JPEG samples inside this project, no dataset download scripts.
    with zipfile.ZipFile(archive) as bundle:
        for member in bundle.namelist():
            if member.lower().endswith('.jpg'):
                dest=ROOT/'logs/yolo_samples/coco128'/Path(member).name
                dest.parent.mkdir(parents=True,exist_ok=True)
                if not dest.exists(): dest.write_bytes(bundle.read(member))
    (ROOT/'checkpoints/yolo/source_manifest.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    print(json.dumps(records,indent=2),flush=True)

if __name__=='__main__': main()

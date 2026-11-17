"""Strict rectangle Labelme importer; never modifies source annotations."""
import json,math,base64,io,hashlib,datetime
from pathlib import Path
from PIL import Image

def import_labelme(dataset,path,group,allow_empty=False):
    path=Path(path);raw=path.read_bytes();data=json.loads(raw)
    if not isinstance(data.get('shapes'),list):raise ValueError('Không phải JSON Labelme')
    p=Path(str(data.get('imagePath') or '').replace('\\','/'));source=p if p.is_absolute() else path.parent/p
    if source.is_file():image=Image.open(source)
    elif data.get('imageData'):image=Image.open(io.BytesIO(base64.b64decode(data['imageData'],validate=True)))
    else:raise ValueError('Thiếu ảnh imagePath/imageData')
    with image:
        if image.getexif().get(274,1)!=1:raise ValueError('EXIF rotation: chuẩn hóa ảnh và gán nhãn lại trước khi import')
        image=image.convert('RGB');w,h=image.size
    if data.get('imageWidth')!=w or data.get('imageHeight')!=h:raise ValueError('Kích thước JSON không khớp ảnh')
    boxes=[]
    for s in data['shapes']:
        if s.get('shape_type')!='rectangle':raise ValueError('Chỉ nhận rectangle; không tự đổi polygon/line thành box')
        if s.get('label') not in dataset.data['classes']:raise ValueError(f'Class chưa khai báo: {s.get("label")}')
        points=s.get('points',[])
        if len(points)!=2 or any(len(p)!=2 for p in points):raise ValueError('Rectangle cần hai điểm')
        v=[float(n) for p in points for n in p]
        if not all(math.isfinite(n) for n in v):raise ValueError('Tọa độ NaN/Inf')
        x1,x2=sorted([v[0],v[2]]);y1,y2=sorted([v[1],v[3]])
        if not (0<=x1<x2<=w and 0<=y1<y2<=h):raise ValueError('Box ngoài ảnh/diện tích bằng 0')
        boxes.append([dataset.data['classes'].index(s['label']),x1,y1,x2,y2])
    if not boxes and not allow_empty:raise ValueError('shapes=[] cần xác nhận ảnh nền, không phải ảnh chưa gán nhãn')
    if not group.strip():raise ValueError('Thiếu group phiên quay')
    digest=hashlib.sha256(str(image.size).encode()+image.tobytes()).hexdigest()
    index=next((i for i,r in enumerate(dataset.data['images']) if r['id']==digest),None)
    if index is not None:
        row=dataset.data['images'][index]
        if row['group']!=group:raise ValueError('Ảnh trùng ở group khác: chặn data leakage')
        if row.get('label_source') and row['label_source']!=str(path.resolve()):raise ValueError('JSON khác cho ảnh trùng: cần giải quyết nhãn mâu thuẫn')
    else:
        image.save(dataset.root/'images'/f'{digest}.png')
        dataset.data['images'].append({'id':digest,'source':str(source),'group':group,'width':w,'height':h,'boxes':[],'reviewed':False,'background':False})
        index=len(dataset.data['images'])-1
    archive=dataset.root/'labelme_sources';archive.mkdir(exist_ok=True)
    copied=archive/(digest+'_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json');copied.write_bytes(raw)
    dataset.data['images'][index].update(label_source=str(path.resolve()),labelme_snapshot=str(copied),labelme_sha256=hashlib.sha256(raw).hexdigest())
    dataset.annotate(index,boxes,background=not boxes)
    return index

"""Đọc dữ liệu gán nhãn LabelMe (train/ và val/) và chuyển sang định dạng YOLO.

Thư mục dữ liệu do người dùng chọn phải có dạng:
    <data>/train/  ảnh + file .json của LabelMe (cùng tên với ảnh)
    <data>/val/    ảnh + file .json của LabelMe

Dữ liệu gốc KHÔNG bị sửa. Bộ dữ liệu YOLO được tạo ở nơi khác (trong thư mục
của lần train) bằng liên kết tới ảnh gốc, nên không tốn thêm dung lượng ảnh.

Chỉ dùng thư viện chuẩn + Pillow.
"""
from pathlib import Path
import json,os

IMAGE_SUFFIXES={'.jpg','.jpeg','.png','.bmp','.webp','.tif','.tiff'}
SPLITS=('train','val')


def list_images(folder):
    return sorted(p for p in Path(folder).rglob('*') if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)


def image_size(path,annotation=None):
    if annotation and annotation.get('imageWidth') and annotation.get('imageHeight'):
        return int(annotation['imageWidth']),int(annotation['imageHeight'])
    from PIL import Image
    with Image.open(path) as image: return image.size


def shape_box(shape):
    """Khung bao (x0, y0, x1, y1) của một shape LabelMe; None nếu không dùng được."""
    points=shape.get('points') or []; kind=shape.get('shape_type','polygon')
    if kind=='circle' and len(points)==2:
        (cx,cy),(px,py)=points; r=((cx-px)**2+(cy-py)**2)**.5
        return cx-r,cy-r,cx+r,cy+r
    if len(points)<2: return None   # 'point' không có diện tích
    xs=[p[0] for p in points]; ys=[p[1] for p in points]
    return min(xs),min(ys),max(xs),max(ys)


def read_annotation(image_path):
    """Trả về (annotation dict | None, lỗi | None) cho một ảnh."""
    path=image_path.with_suffix('.json')
    if not path.is_file(): return None,None
    try: return json.loads(path.read_text(encoding='utf-8')),None
    except Exception as error: return None,f'{path.name}: {error}'


def scan(data_dir):
    """Kiểm tra thư mục dữ liệu. Không ghi gì ra đĩa."""
    root=Path(data_dir).expanduser()
    report={'path':str(root),'ok':False,'errors':[],'warnings':[],'splits':{},'labels':{},'label_order':[]}
    if not root.is_dir(): report['errors'].append('Thư mục không tồn tại.'); return report
    for split in SPLITS:
        if not (root/split).is_dir(): report['errors'].append(f'Thiếu thư mục con "{split}".')
    if report['errors']: return report
    for split in SPLITS:
        images=list_images(root/split)
        info={'images':len(images),'labelled':0,'without_json':0,'empty_json':0,'boxes':0,'broken_json':[],'labels':{},'shape_types':{},
              'tiny_boxes':0,'examples_without_json':[]}
        for image in images:
            annotation,error=read_annotation(image)
            if error: info['broken_json'].append(error); continue
            if annotation is None:
                info['without_json']+=1
                if len(info['examples_without_json'])<5: info['examples_without_json'].append(image.name)
                continue
            shapes=annotation.get('shapes') or []
            if not shapes: info['empty_json']+=1; continue
            info['labelled']+=1
            try: width,height=image_size(image,annotation)
            except Exception as error: info['broken_json'].append(f'{image.name}: {error}'); continue
            for shape in shapes:
                label=str(shape.get('label','')).strip(); box=shape_box(shape)
                if not label or box is None: continue
                info['labels'][label]=info['labels'].get(label,0)+1; info['boxes']+=1
                kind=shape.get('shape_type','polygon'); info['shape_types'][kind]=info['shape_types'].get(kind,0)+1
                if (box[2]-box[0])<.01*width or (box[3]-box[1])<.01*height: info['tiny_boxes']+=1
        report['splits'][split]=info
        for label,count in info['labels'].items():
            report['labels'].setdefault(label,{'train':0,'val':0})[split]=count
    train,val=report['splits']['train'],report['splits']['val']
    if not train['images']: report['errors'].append('Thư mục train không có ảnh.')
    if not val['images']: report['errors'].append('Thư mục val không có ảnh.')
    if train['images'] and not train['boxes']: report['errors'].append('Thư mục train chưa có nhãn nào (không tìm thấy file .json của LabelMe cạnh ảnh).')
    if val['images'] and not val['boxes']: report['warnings'].append('Thư mục val chưa có nhãn: sẽ không tính được độ chính xác (mAP).')
    for split,info in report['splits'].items():
        if info['without_json']: report['warnings'].append(f'{split}: {info["without_json"]} ảnh không có file .json (sẽ được coi là ảnh nền, không có vật thể).')
        if info['broken_json']: report['warnings'].append(f'{split}: {len(info["broken_json"])} file .json bị lỗi và sẽ bị bỏ qua.')
        if info['tiny_boxes']: report['warnings'].append(f'{split}: {info["tiny_boxes"]} khung rất nhỏ (dưới 1% kích thước ảnh), mô hình khó học.')
        other={k:v for k,v in info['shape_types'].items() if k!='rectangle'}
        if other: report['warnings'].append(f'{split}: có shape không phải hình chữ nhật {other}; sẽ dùng khung bao quanh của chúng.')
    for label,count in report['labels'].items():
        if count['train'] and not count['val']: report['warnings'].append(f'Nhãn "{label}" có trong train nhưng không có trong val.')
        if count['val'] and not count['train']: report['warnings'].append(f'Nhãn "{label}" chỉ có trong val, không có trong train: mô hình không học được.')
        if 0<count['train']<20: report['warnings'].append(f'Nhãn "{label}" chỉ có {count["train"]} khung trong train (nên có vài trăm).')
    if train['images'] and val['images'] and val['images']<.05*train['images']:
        report['warnings'].append('Tập val rất nhỏ so với train (dưới 5%); nên dành khoảng 10-20% ảnh cho val.')
    # Thứ tự nhãn mặc định: nhiều khung nhất trước (người dùng có thể sửa trên giao diện).
    report['label_order']=sorted(report['labels'],key=lambda k:(-report['labels'][k]['train'],k))
    report['ok']=not report['errors']
    return report


def convert(data_dir,output_dir,class_names,include_background=True):
    """Tạo bộ dữ liệu YOLO tại output_dir. Trả về thống kê và đường dẫn data.yaml.

    class_names: danh sách tên nhãn theo đúng thứ tự id (0, 1, 2, ...). Nhãn
    trong LabelMe không nằm trong danh sách này sẽ bị bỏ qua.
    """
    root=Path(data_dir).expanduser().resolve(); output=Path(output_dir).resolve()
    index={name:i for i,name in enumerate(class_names)}
    if len(index)!=len(class_names) or not class_names: raise ValueError('Danh sách nhãn rỗng hoặc bị trùng tên.')
    stats={'splits':{},'ignored_labels':{},'class_names':list(class_names)}
    for split in SPLITS:
        images_dir=output/'images'/split; labels_dir=output/'labels'/split
        images_dir.mkdir(parents=True,exist_ok=True); labels_dir.mkdir(parents=True,exist_ok=True)
        info={'images':0,'boxes':0,'background_images':0,'skipped_images':0,'per_class':{name:0 for name in class_names}}
        for number,image in enumerate(list_images(root/split)):
            annotation,error=read_annotation(image)
            if error: info['skipped_images']+=1; continue
            lines=[]
            if annotation is not None:
                try: width,height=image_size(image,annotation)
                except Exception: info['skipped_images']+=1; continue
                for shape in annotation.get('shapes') or []:
                    label=str(shape.get('label','')).strip(); box=shape_box(shape)
                    if box is None or not label: continue
                    if label not in index:
                        stats['ignored_labels'][label]=stats['ignored_labels'].get(label,0)+1; continue
                    x0,y0,x1,y1=[max(0.,min(float(limit),float(v))) for v,limit in zip(box,(width,height,width,height))]
                    if x1-x0<1 or y1-y0<1: continue
                    lines.append(f'{index[label]} {(x0+x1)/2/width:.6f} {(y0+y1)/2/height:.6f} {(x1-x0)/width:.6f} {(y1-y0)/height:.6f}')
                    info['per_class'][label]+=1
            if not lines:
                if not include_background: info['skipped_images']+=1; continue
                info['background_images']+=1
            # Tên duy nhất (ảnh có thể nằm trong thư mục con trùng tên); ảnh chỉ là liên kết, không sao chép.
            stem=f'{number:06d}_{image.stem}'; link=images_dir/(stem+image.suffix.lower())
            if link.is_symlink() or link.exists(): link.unlink()
            os.symlink(image,link)
            (labels_dir/(stem+'.txt')).write_text('\n'.join(lines)+('\n' if lines else ''))
            info['images']+=1; info['boxes']+=len(lines)
        stats['splits'][split]=info
    yaml_path=output/'data.yaml'
    names='\n'.join(f'  {i}: {json.dumps(name,ensure_ascii=False)}' for i,name in enumerate(class_names))
    yaml_path.write_text(f'path: {json.dumps(str(output))}\ntrain: images/train\nval: images/val\nnames:\n{names}\n',encoding='utf-8')
    stats['data_yaml']=str(yaml_path)
    return stats

"""Dataset preparation only. No model training calls."""
from pathlib import Path
import hashlib,json,random,shutil,datetime
from PIL import Image,ImageOps
ROOT=Path(__file__).resolve().parent

def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        backup=path.parent/'backups'/(path.name+'.'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,backup)
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8');tmp.replace(path)

class Dataset:
    def __init__(self,root=ROOT):
        self.root=Path(root);self.file=self.root/'dataset.json'
        for folder in ['images','labels','exports','plans','backups']: (self.root/folder).mkdir(parents=True,exist_ok=True)
        self.data=json.loads(self.file.read_text(encoding='utf-8')) if self.file.exists() else {'classes':['person','machine','box','shelf'],'images':[]}

    def save(self):write_json(self.file,self.data)

    def classes(self,names):
        names=[n.strip() for n in names if n.strip()]
        if not names or len(set(names))!=len(names):raise ValueError('Class phải có tên và không trùng nhau')
        if any(x['reviewed'] for x in self.data['images']) and names!=self.data['classes']:
            raise ValueError('Đã có nhãn: không đổi thứ tự/class để tránh sai mapping. Tạo dataset riêng nếu cần đổi.')
        self.data['classes']=names;self.save()

    def add(self,path,group):
        if not group.strip():raise ValueError('Nhập tên phiên quay / scene group')
        with Image.open(path) as source:image=ImageOps.exif_transpose(source).convert('RGB')
        digest=hashlib.sha256(str(image.size).encode()+image.tobytes()).hexdigest()
        if any(x['id']==digest for x in self.data['images']):return False
        image.save(self.root/'images'/f'{digest}.png')
        self.data['images'].append({'id':digest,'source':str(path),'group':group.strip(),'width':image.width,'height':image.height,
                                    'boxes':[],'reviewed':False,'background':False})
        self.save();return True

    def annotate(self,index,boxes,background=False):
        row=self.data['images'][index]
        if background and boxes:raise ValueError('Ảnh background không được có box')
        if not boxes and not background:raise ValueError('Vẽ box hoặc xác nhận ảnh nền trống')
        for box in boxes:
            c,x1,y1,x2,y2=box
            if c not in range(len(self.data['classes'])) or not (0<=x1<x2<=row['width'] and 0<=y1<y2<=row['height']):
                raise ValueError('Box/class không hợp lệ')
        row.update(boxes=boxes,reviewed=True,background=background);self.save()

    def audit(self):
        errors=[];counts={n:0 for n in self.data['classes']};groups=set()
        for i,row in enumerate(self.data['images']):
            groups.add(row['group'])
            if not row['reviewed']:errors.append(f'Ảnh {i+1} chưa kiểm tra/gán nhãn')
            if not (self.root/'images'/f"{row['id']}.png").exists():errors.append(f'Ảnh {i+1} thiếu file')
            for c,*_ in row['boxes']:counts[self.data['classes'][c]]+=1
        if not self.data['images']:errors.append('Chưa có ảnh dataset thực tế')
        if len(groups)<3:errors.append('Cần ít nhất 3 group độc lập để tách train/val/test')
        for name,count in counts.items():
            if not count:errors.append(f'Class {name} chưa có box')
        return {'images':len(self.data['images']),'groups':len(groups),'instances':counts,'errors':errors,
                'note':'Không có ngưỡng ảnh tối thiểu bảo đảm chất lượng. Cần thêm dữ liệu đa dạng và đánh giá riêng.'}

    def prepare(self,settings):
        audit=self.audit()
        if audit['errors']:raise ValueError('\n'.join(audit['errors'][:15]))
        groups=sorted({r['group'] for r in self.data['images']});random.Random(42).shuffle(groups)
        nv=max(1,round(len(groups)*.15));nt=max(1,round(len(groups)*.15))
        splits={'val':groups[:nv],'test':groups[nv:nv+nt],'train':groups[nv+nt:]}
        tag=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');target=self.root/'exports'/tag;target.mkdir()
        mapping={};coverage={}
        for split,selected in splits.items():
            (target/'images'/split).mkdir(parents=True);(target/'labels'/split).mkdir(parents=True)
            coverage[split]=set()
            for row in self.data['images']:
                if row['group'] not in selected:continue
                key=row['id'];w,h=row['width'],row['height'];mapping[key]=split
                shutil.copy2(self.root/'images'/f'{key}.png',target/'images'/split/f'{key}.png')
                lines=[]
                for c,x1,y1,x2,y2 in row['boxes']:
                    coverage[split].add(c);lines.append(f'{c} {(x1+x2)/2/w:.8f} {(y1+y2)/2/h:.8f} {(x2-x1)/w:.8f} {(y2-y1)/h:.8f}')
                (target/'labels'/split/f'{key}.txt').write_text('\n'.join(lines),encoding='utf-8')
        # JSON is valid YAML and avoids unsafe YAML construction from class names.
        write_json(target/'data.yaml',{'path':str(target),'train':'images/train','val':'images/val','test':'images/test','names':self.data['classes']})
        write_json(target/'split_manifest.json',{'groups':splits,'image_split':mapping,'seed':42})
        architecture=ROOT.parent/'.venv/yolo/Lib/site-packages/ultralytics/cfg/models/11/yolo11.yaml'
        if not architecture.exists():raise ValueError('Thiếu cấu hình kiến trúc YOLO11 trong môi trường hiện có')
        # Architecture definition only, never pretrained parameters or .pt weights.
        source=architecture.read_text(encoding='utf-8').replace('nc: 80',f'nc: {len(self.data["classes"])}')
        (target/'custom_yolo11n.yaml').write_text(source+'\nscale: n\n',encoding='utf-8')
        missing={s:[self.data['classes'][c] for c in range(len(self.data['classes'])) if c not in coverage[s]] for s in splits}
        plan={'status':'WAITING_FOR_USER_TRAINING_APPROVAL','training_started':False,'initialization':'random_from_architecture',
              'pretrained':False,'model':str(target/'custom_yolo11n.yaml'),'data':str(target/'data.yaml'),**settings,
              'class_coverage_missing':missing,'ready_for_review':not any(missing.values()),
              'backend_note':'Existing inference environment is CPU-only. RTX 5070 training environment is NOT prepared/validated yet.',
              'export_after_training':['detector.onnx','classes.json','inference_config.json','model_manifest.json','evaluation_report.json']}
        write_json(target/'training_plan.json',plan);write_json(target/'classes.json',self.data['classes'])
        return target,plan

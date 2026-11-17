"""Meaningful data/GUI smoke tests only; never trains a network."""
import tempfile,json,tkinter as tk
from pathlib import Path
from PIL import Image
from dataset import Dataset
from labelme_import import import_labelme

def main():
    with tempfile.TemporaryDirectory(dir=Path(__file__).parent) as folder:
        root=Path(folder);ds=Dataset(root);ds.classes(['person'])
        for i in range(3):
            image=root/f'source_{i}.png';Image.new('RGB',(100,80),(50*i,100,130)).save(image)
            label=root/f'source_{i}.json'
            annotation={'imagePath':image.name,'imageWidth':100,'imageHeight':80,'shapes':[{'label':'person','shape_type':'rectangle','points':[[60,70],[10,10]]}]}
            label.write_text(json.dumps(annotation))
            original=label.read_bytes()
            assert import_labelme(ds,label,f'independent_fixture_{i}')==i
            assert label.read_bytes()==original
            assert not ds.add(image,'duplicate_other_group')
            ds.annotate(i,[[0,10.,10.,60.,70.]])
        try:ds.annotate(0,[[0,-1,0,10,10]])
        except ValueError:pass
        else:raise AssertionError('Invalid bbox accepted')
        assert not ds.audit()['errors']
        target,plan=ds.prepare({'imgsz':320,'epochs':100,'batch':4})
        assert plan['pretrained'] is False and plan['training_started'] is False and plan['ready_for_review']
        manifest=json.loads((target/'split_manifest.json').read_text());assert len(set(manifest['image_split'].values()))==3
        assert 'scale: n' in (target/'custom_yolo11n.yaml').read_text()
        assert not list(root.rglob('*.pt'))
        text=label.read_text();bad=json.loads(text);bad['shapes'][0]['shape_type']='polygon';label.write_text(json.dumps(bad))
        try:import_labelme(ds,label,'independent_fixture_2')
        except ValueError:pass
        else:raise AssertionError('Polygon accepted')
        bad=json.loads(text);bad['shapes']=[];label.write_text(json.dumps(bad))
        try:import_labelme(ds,label,'independent_fixture_2')
        except ValueError:pass
        else:raise AssertionError('Empty labels silently accepted')
        import_labelme(ds,label,'independent_fixture_2',allow_empty=True)
        assert ds.data['images'][2]['background']
        label.write_text(text)
        try:import_labelme(ds,label,'different_group')
        except ValueError:pass
        else:raise AssertionError('Duplicate leaked across groups')
        from app import App
        window=tk.Tk();app=App(window);app.ds=ds;window.update();app.load();window.update()
        assert app.canvas.find_all(),'No image/annotation canvas items'
        assert app.boxes and app.photo
        app.dirty=False;window.destroy()
    print('PASS import/dedup/invalid-box rejection/group split/YOLO labels/random-init plan/GUI render; NO TRAINING')

if __name__=='__main__':main()

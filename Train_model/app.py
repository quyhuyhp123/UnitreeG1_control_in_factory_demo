"""Read-only Labelme review and training preparation; no annotation editor or train execution."""
import tkinter as tk
from tkinter import ttk,filedialog,messagebox,simpledialog
from pathlib import Path
import json
from PIL import Image,ImageTk
from dataset import Dataset,ROOT
from labelme_import import import_labelme

class App:
    def __init__(self,window):
        self.window=window;window.title('G1 Train_model | Labelme -> Training | TRAINING LOCKED');window.geometry('1250x850')
        self.ds=Dataset();self.index=0;self.boxes=[];self.dirty=False
        toolbar=ttk.Frame(window);toolbar.pack(fill='x')
        for title,fn in [('Class',self.edit_classes),('Import Labelme JSON',self.import_files),('Import folder Labelme',self.import_folder),('Audit dataset',self.audit),('Prepare training',self.prepare)]:
            ttk.Button(toolbar,text=title,command=lambda f=fn:self.guard(f)).pack(side='left',padx=5,pady=8)
        ttk.Label(window,text='Read-only preview. Edit labels in Labelme, then re-import the JSON with the same group. Training is LOCKED.').pack()
        body=ttk.Panedwindow(window,orient='horizontal');body.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(body,bg='#20262e');body.add(self.canvas,weight=4)
        side=ttk.Frame(body);body.add(side,weight=1)
        self.listbox=tk.Listbox(side);self.listbox.pack(fill='both',expand=True)
        ttk.Label(side,text='Box preview only. No label editing.').pack()
        controls=ttk.Frame(window);controls.pack(fill='x')
        ttk.Button(controls,text='Previous',command=lambda:self.navigate(-1)).pack(side='left')
        ttk.Button(controls,text='Next',command=lambda:self.navigate(1)).pack(side='left')
        ttk.Button(controls,text='TRAIN - waiting for your decision',state='disabled').pack(side='right')
        self.status=ttk.Label(window);self.status.pack(fill='x')
        self.canvas.bind('<Configure>',lambda e:self.draw());self.load()

    def import_files(self):self.import_paths(filedialog.askopenfilenames(filetypes=[('Labelme JSON','*.json')]))
    def import_folder(self):
        folder=filedialog.askdirectory(title='One capture group: Labelme images + JSON')
        if not folder:return
        images=[p for p in Path(folder).iterdir() if p.suffix.lower() in ['.png','.jpg','.jpeg','.bmp','.webp']]
        missing=[p.name for p in images if not p.with_suffix('.json').exists()]
        if missing:messagebox.showwarning('Missing JSON - NOT background','\n'.join(missing[:20]))
        paths=sorted(Path(folder).glob('*.json'))
        if not paths:raise ValueError('No Labelme JSON in folder')
        self.import_paths(paths)
    def import_paths(self,paths):
        if not paths:return
        group=simpledialog.askstring('Group','Capture session / scene group (same session = same group):')
        if not group:return
        allow=messagebox.askyesno('Empty labels','Allow shapes=[] as confirmed background? Only Yes if these empty labels were deliberately reviewed.')
        errors=[];count=0
        for path in paths:
            try:import_labelme(self.ds,path,group,allow);count+=1
            except Exception as e:errors.append(f'{Path(path).name}: {e}')
        self.load();messagebox.showinfo('Import',f'Imported/updated: {count}; errors: {len(errors)}\n'+'\n'.join(errors[:15]))
    def guard(self,fn):
        try:fn()
        except Exception as error:messagebox.showerror('Thông báo',str(error))


    def row(self):return self.ds.data['images'][self.index] if self.ds.data['images'] else None


    def load(self):
        row=self.row();self.boxes=[list(b) for b in row['boxes']] if row else [];self.dirty=False;self.draw()


    def draw(self):
        self.canvas.delete('all');self.listbox.delete(0,'end');row=self.row()
        if not row:self.status.config(text='Dataset trống — gán rectangle trong Labelme rồi import JSON');return
        w=max(50,self.canvas.winfo_width());h=max(50,self.canvas.winfo_height())
        self.scale=min(w/row['width'],h/row['height']);self.offset=((w-row['width']*self.scale)/2,(h-row['height']*self.scale)/2)
        with Image.open(self.ds.root/'images'/f"{row['id']}.png") as image:
            self.photo=ImageTk.PhotoImage(image.resize((max(1,int(row['width']*self.scale)),max(1,int(row['height']*self.scale)))))
        self.canvas.create_image(*self.offset,anchor='nw',image=self.photo)
        for i,(c,x1,y1,x2,y2) in enumerate(self.boxes):
            x,y=self.offset;coords=[x+x1*self.scale,y+y1*self.scale,x+x2*self.scale,y+y2*self.scale]
            self.canvas.create_rectangle(*coords,outline='#00ff88',width=2)
            self.canvas.create_text(coords[0]+3,coords[1]+10,text=f'{i+1}: {self.ds.data["classes"][c]}',anchor='w',fill='#00ff88')
            self.listbox.insert('end',f'{i+1}. {self.ds.data["classes"][c]}')
        self.status.config(text=f'{self.index+1}/{len(self.ds.data["images"])} | group: {row["group"]} | {"đã kiểm tra" if row["reviewed"] else "CHƯA GÁN NHÃN"} | {row["width"]}×{row["height"]}')


    def navigate(self,direction):
        if self.ds.data['images']:self.index=(self.index+direction)%len(self.ds.data['images']);self.load()


    def edit_classes(self):
        names=simpledialog.askstring('Class','Tên class, ngăn cách bằng dấu phẩy:',initialvalue=','.join(self.ds.data['classes']))
        if names is not None:
            if self.dirty:raise ValueError('Lưu nhãn hiện tại trước khi đổi class')
            self.ds.classes(names.split(','));self.draw()


    def audit(self):messagebox.showinfo('Dataset audit',json.dumps(self.ds.audit(),indent=2,ensure_ascii=False))


    def prepare(self):
        if self.dirty:raise ValueError('Lưu nhãn trước khi chuẩn bị')
        imgsz=simpledialog.askinteger('Training plan','imgsz (bội số 32):',initialvalue=320,minvalue=128,maxvalue=1280)
        if imgsz is None:return
        if imgsz%32:raise ValueError('imgsz phải là bội số 32')
        epochs=simpledialog.askinteger('Training plan','Số epochs dự kiến:',initialvalue=100,minvalue=1,maxvalue=1000)
        if epochs is None:return
        batch=simpledialog.askinteger('Training plan','Batch dự kiến (chưa benchmark VRAM):',initialvalue=4,minvalue=1,maxvalue=64)
        if batch is None:return
        path,plan=self.ds.prepare({'imgsz':imgsz,'epochs':epochs,'batch':batch,'seed':42})
        messagebox.showinfo('Đã chuẩn bị — CHƯA TRAIN',f'{path}\n\nThiếu coverage: {plan["class_coverage_missing"]}\nMôi trường GPU training chưa được cài/kiểm thử. Chờ quyết định của bạn.')


if __name__=='__main__':
    window=tk.Tk();App(window);window.mainloop()

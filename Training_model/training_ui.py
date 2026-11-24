#!/usr/bin/env python3
"""Giao diện huấn luyện lại YOLO với dữ liệu gán nhãn bằng LabelMe (Tkinter).

Mở bằng:  ./run_ui.sh        (hoặc: python3 training_ui.py)

Các tab:
  1. Dữ liệu      chọn thư mục chứa train/ và val/, kiểm tra nhãn LabelMe
  2. Huấn luyện   tên nhãn, batch, epoch, imgsz ... -> Bắt đầu / Dừng / Train tiếp
  3. Biểu đồ      loss và độ chính xác theo epoch, tự cập nhật khi đang train
  4. Kiểm tra     mở ảnh và xem mô hình phát hiện gì
  5. Kết quả      danh sách các lần train, ảnh đánh giá, xuất ONNX

Giao diện chỉ dùng thư viện chuẩn của Python + Pillow. Việc nặng (train, chạy
thử, xuất mô hình) chạy ở tiến trình riêng qua ml_python.sh, dùng PyTorch/CUDA
có sẵn của Isaac Sim và Ultralytics cài trong .deps. Không khởi động Isaac Sim,
không điều khiển robot, không sửa cấu hình của dự án mô phỏng.
"""
from pathlib import Path
import base64,csv,io,json,os,queue,re,shutil,signal,subprocess,sys,threading,time
import tkinter as tk
from tkinter import filedialog,messagebox,ttk
import tkinter.font as tkfont

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from labelme_to_yolo import scan,IMAGE_SUFFIXES

RUNS=HERE/'runs'; WEIGHTS=HERE/'weights'; DEPS=HERE/'.deps'; SETTINGS=HERE/'ui_settings.json'; ML_PYTHON=HERE/'ml_python.sh'
RUNS.mkdir(exist_ok=True); WEIGHTS.mkdir(exist_ok=True)
SAFE_NAME=re.compile(r'[^A-Za-z0-9_\-]+'); ANSI=re.compile(r'\x1b\[[0-9;?]*[A-Za-z]')
DOWNLOADABLE=['yolo11n.pt','yolo11s.pt','yolo11m.pt','yolo11l.pt']
COLOURS=['#2563eb','#dc2626','#059669','#d97706','#7c3aed','#0891b2','#be185d','#4d7c0f']
STATE_TEXT={'starting':'Đang khởi động','preparing':'Đang chuẩn bị dữ liệu','training':'Đang train','finished':'Hoàn tất',
            'failed':'Lỗi','stopped':'Đã dừng','unknown':'Không rõ'}


# ---------------------------------------------------------------- dữ liệu của các lần train
from platform_runtime import ml_command,worker_environment,process_options,stop_process,dependencies_ready,cuda_available

def deps_ready(): return dependencies_ready()

def read_results(run):
    """results.csv của Ultralytics -> {tên cột: [giá trị theo epoch]}."""
    file=run/'results.csv'; columns={}
    if not file.is_file(): return columns
    try:
        with file.open(newline='') as handle:
            for row in csv.DictReader(handle):
                for key,value in row.items():
                    if key is None: continue
                    try: columns.setdefault(key.strip(),[]).append(float(value))
                    except (TypeError,ValueError): columns.setdefault(key.strip(),[]).append(None)
    except Exception: return {}
    return columns

def read_json(path,default=None):
    try: return json.loads(Path(path).read_text(encoding='utf-8'))
    except Exception: return {} if default is None else default

def run_summary(run,active_name=None):
    config=read_json(run/'config.json'); state=read_json(run/'status.json').get('state','unknown')
    if run.name!=active_name and state in ('starting','preparing','training'): state='stopped'
    results=read_results(run); done=len(results.get('epoch',[]))
    last={k:v[-1] for k,v in results.items() if v}
    metric=lambda prefix:next((last[k] for k in last if k.startswith(prefix)),None)
    weights=run/'weights'
    return {'name':run.name,'path':run,'state':state,'config':config,'epochs':config.get('epochs'),'epochs_done':done,
        'precision':metric('metrics/precision'),'recall':metric('metrics/recall'),'map50':metric('metrics/mAP50('),'map50_95':metric('metrics/mAP50-95'),
        'has_best':(weights/'best.pt').is_file(),'has_last':(weights/'last.pt').is_file(),
        'onnx':sorted(p.name for p in weights.glob('*.onnx')) if weights.is_dir() else [],'modified':run.stat().st_mtime}

def list_runs(): return sorted((p for p in RUNS.iterdir() if p.is_dir()),key=lambda p:-p.stat().st_mtime)

def gpu_info():
    try:
        out=subprocess.run(['nvidia-smi','--query-gpu=index,name,memory.used,memory.total,utilization.gpu','--format=csv,noheader,nounits'],
                           capture_output=True,text=True,encoding="utf-8",errors="replace",timeout=5).stdout
        return [[v.strip() for v in line.split(',')] for line in out.splitlines() if line.strip()]
    except Exception: return []

def fmt(value,digits=3): return '-' if value is None else f'{value:.{digits}f}'

def photo_from_image(image,max_width,max_height):
    """PIL image -> (tk.PhotoImage, tỉ lệ thu nhỏ). Không cần ImageTk."""
    scale=min(max_width/image.width,max_height/image.height,1.0) if max_width>1 and max_height>1 else 1.0
    if scale<1.0: image=image.resize((max(1,int(image.width*scale)),max(1,int(image.height*scale))))
    buffer=io.BytesIO(); image.save(buffer,format='PNG')
    return tk.PhotoImage(data=base64.b64encode(buffer.getvalue())),scale


# ---------------------------------------------------------------- vẽ biểu đồ trên Canvas
def draw_chart(canvas,title,series,log_scale=False):
    """series: {tên: [giá trị theo epoch]} (None = thiếu)."""
    canvas.delete('all'); width,height=canvas.winfo_width(),canvas.winfo_height()
    if width<80 or height<80: return
    left,right,top,bottom=58,14,30,30
    canvas.create_text(left,10,text=title,anchor='nw',font=('TkDefaultFont',10,'bold'),fill='#111827')
    values=[v for data in series.values() for v in data if v is not None]
    count=max((len(d) for d in series.values()),default=0)
    if not values or count==0:
        canvas.create_text(width/2,height/2,text='Chưa có dữ liệu',fill='#6b7280'); return
    low,high=min(values),max(values)
    if high-low<1e-12: low,high=low-.5,high+.5
    pad=(high-low)*.06; low-=pad; high+=pad
    plot_w,plot_h=width-left-right,height-top-bottom
    def px(i): return left+(plot_w*i/max(1,count-1) if count>1 else plot_w/2)
    def py(v): return top+plot_h*(1-(v-low)/(high-low))
    canvas.create_rectangle(left,top,left+plot_w,top+plot_h,outline='#d1d5db',fill='#ffffff')
    for k in range(5):
        v=low+(high-low)*k/4; y=py(v)
        canvas.create_line(left,y,left+plot_w,y,fill='#eef0f3')
        canvas.create_text(left-6,y,text=f'{v:.3g}',anchor='e',fill='#4b5563',font=('TkDefaultFont',8))
    ticks=min(count,8)
    for k in range(ticks):
        i=round((count-1)*k/max(1,ticks-1)) if count>1 else 0
        canvas.create_text(px(i),top+plot_h+12,text=str(i+1),fill='#4b5563',font=('TkDefaultFont',8))
    canvas.create_text(left+plot_w,top+plot_h+24,text='epoch',anchor='e',fill='#6b7280',font=('TkDefaultFont',8))
    legend_x=left+8
    for index,(name,data) in enumerate(series.items()):
        colour=COLOURS[index%len(COLOURS)]; points=[]
        for i,v in enumerate(data):
            if v is None:
                if len(points)>=4: canvas.create_line(*points,fill=colour,width=2)
                points=[]
            else: points+=[px(i),py(v)]
        if len(points)>=4: canvas.create_line(*points,fill=colour,width=2)
        elif len(points)==2: canvas.create_oval(points[0]-3,points[1]-3,points[0]+3,points[1]+3,fill=colour,outline='')
        last=next((v for v in reversed(data) if v is not None),None)
        label=f'{name}: {last:.4g}' if last is not None else name
        canvas.create_line(legend_x,top+10,legend_x+16,top+10,fill=colour,width=3)
        item=canvas.create_text(legend_x+20,top+10,text=label,anchor='w',fill='#111827',font=('TkDefaultFont',8))
        legend_x=canvas.bbox(item)[2]+14


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Huấn luyện YOLO - dữ liệu LabelMe'); self.geometry('1320x880'); self.minsize(1080,720)
        try: ttk.Style(self).theme_use('clam')
        except tk.TclError: pass
        for name in ('TkDefaultFont','TkTextFont','TkMenuFont','TkHeadingFont'):
            try: tkfont.nametofont(name).configure(size=10)
            except tk.TclError: pass
        style=ttk.Style(self)
        style.configure('Title.TLabel',font=('TkDefaultFont',12,'bold')); style.configure('Hint.TLabel',foreground='#6b7280')
        style.configure('Bad.TLabel',foreground='#b91c1c'); style.configure('Good.TLabel',foreground='#047857')
        style.configure('Primary.TButton',font=('TkDefaultFont',10,'bold'))
        self.settings=read_json(SETTINGS)
        self.report=None; self.process=None; self.active_run=None; self.log_offset=0; self.log_tail=''; self.train_started=None
        self.setup_process=None; self.predict_process=None; self.predict_busy=False; self.predict_pending=False
        self.test_images=[]; self.test_index=0; self.test_photo=None; self.test_image=None; self.test_result=None; self.plot_photo=None
        self.ui_queue=queue.Queue()

        self.banner=ttk.Frame(self,padding=(10,6)); self.banner.pack(fill='x')
        self.notebook=ttk.Notebook(self); self.notebook.pack(fill='both',expand=True,padx=8,pady=(0,4))
        self.tab_data=ttk.Frame(self.notebook,padding=10); self.tab_train=ttk.Frame(self.notebook,padding=10)
        self.tab_chart=ttk.Frame(self.notebook,padding=10); self.tab_test=ttk.Frame(self.notebook,padding=10); self.tab_runs=ttk.Frame(self.notebook,padding=10)
        for tab,text in ((self.tab_data,' 1. Dữ liệu '),(self.tab_train,' 2. Huấn luyện '),(self.tab_chart,' 3. Biểu đồ loss '),
                         (self.tab_test,' 4. Kiểm tra mô hình '),(self.tab_runs,' 5. Kết quả & Xuất mô hình ')):
            self.notebook.add(tab,text=text)
        self.status_var=tk.StringVar(); ttk.Label(self,textvariable=self.status_var,anchor='w',padding=(10,3),relief='sunken').pack(fill='x')
        self.build_banner(); self.build_data_tab(); self.build_train_tab(); self.build_chart_tab(); self.build_test_tab(); self.build_runs_tab()
        self.notebook.bind('<<NotebookTabChanged>>',lambda e:self.on_tab_changed())
        self.protocol('WM_DELETE_WINDOW',self.on_close)
        self.refresh_runs(); self.refresh_system(); self.after(300,self.tick); self.after(200,self.pump)

    # ------------------------------------------------------------ tiện ích chung
    def background(self,work,done):
        """Chạy work() ở luồng phụ, rồi gọi done(kết quả | Exception) ở luồng giao diện."""
        def runner():
            try: result=work()
            except Exception as error: result=error
            self.ui_queue.put((done,result))
        threading.Thread(target=runner,daemon=True).start()

    def pump(self):
        try:
            while True:
                done,result=self.ui_queue.get_nowait(); done(result)
        except queue.Empty: pass
        self.after(120,self.pump)

    def save_settings(self):
        try: SETTINGS.write_text(json.dumps(self.settings,ensure_ascii=False,indent=2),encoding='utf-8')
        except OSError: pass

    def training_running(self): return self.process is not None and self.process.poll() is None

    # ------------------------------------------------------------ dải thông báo môi trường
    def build_banner(self):
        for child in self.banner.winfo_children(): child.destroy()
        if deps_ready():
            ttk.Label(self.banner,text=('Windows ML backend: '+('CUDA available' if cuda_available() else 'CPU only') if os.name=='nt' else 'Linux .deps + Isaac ML environment available'),style='Good.TLabel').pack(side='left')
        else:
            ttk.Label(self.banner,text='Chưa cài môi trường train (Ultralytics). Cần cài một lần trước khi train hoặc chạy thử mô hình.',style='Bad.TLabel').pack(side='left')
            self.setup_button=ttk.Button(self.banner,text=('Check environment' if os.name=='nt' else 'Setup environment'),command=self.run_setup); self.setup_button.pack(side='left',padx=10)

    def run_setup(self):
        if os.name=="nt":
            messagebox.showinfo("Windows environment", "Use Training_model/.venv or set G1_TRAINING_PYTHON to an isolated Python with PyTorch and Ultralytics.")
            return
        if self.setup_process is not None and self.setup_process.poll() is None: return
        if not messagebox.askyesno('Cài môi trường','Sẽ tải Ultralytics (~230 MB) vào Training_model/.deps và trọng số yolo11n.pt.\n'
                                   'Không cài hay sửa gì trong Isaac Sim. Cần mạng Internet.\n\nTiếp tục?'): return
        self.notebook.select(self.tab_train); self.log_clear(); self.log_append('== Cài môi trường ==\n')
        self.setup_log=HERE/'setup.log'; handle=self.setup_log.open('wb')
        self.setup_process=subprocess.Popen(['bash',str(HERE/'setup_env.sh')],stdout=handle,stderr=subprocess.STDOUT,cwd=str(HERE))
        self.setup_offset=0; self.setup_button.configure(state='disabled')

    # ------------------------------------------------------------ TAB 1: dữ liệu
    def build_data_tab(self):
        tab=self.tab_data
        ttk.Label(tab,text='Bước 1 - Chọn thư mục dữ liệu',style='Title.TLabel').pack(anchor='w')
        ttk.Label(tab,style='Hint.TLabel',text='Thư mục phải chứa 2 thư mục con "train" và "val". Trong mỗi thư mục là ảnh và file .json của LabelMe (cùng tên với ảnh). '
                  'Dữ liệu gốc không bị sửa.').pack(anchor='w',pady=(2,8))
        row=ttk.Frame(tab); row.pack(fill='x')
        self.data_var=tk.StringVar(value=self.settings.get('data_dir',''))
        ttk.Entry(row,textvariable=self.data_var).pack(side='left',fill='x',expand=True)
        ttk.Button(row,text='Chọn thư mục...',command=self.choose_data).pack(side='left',padx=6)
        ttk.Button(row,text='Kiểm tra dữ liệu',style='Primary.TButton',command=self.scan_data).pack(side='left')
        self.data_summary=tk.StringVar(value='Chưa kiểm tra.')
        ttk.Label(tab,textvariable=self.data_summary,wraplength=1250,justify='left').pack(anchor='w',pady=8)
        middle=ttk.Frame(tab); middle.pack(fill='both',expand=True)
        left=ttk.LabelFrame(middle,text='Nhãn tìm thấy trong LabelMe',padding=6); left.pack(side='left',fill='both',expand=True)
        self.label_tree=ttk.Treeview(left,columns=('label','train','val'),show='headings',height=10)
        for column,text,width in (('label','Tên nhãn',240),('train','Số khung (train)',130),('val','Số khung (val)',130)):
            self.label_tree.heading(column,text=text); self.label_tree.column(column,width=width,anchor='w' if column=='label' else 'e')
        self.label_tree.pack(fill='both',expand=True)
        right=ttk.LabelFrame(middle,text='Thống kê từng tập',padding=6); right.pack(side='left',fill='both',expand=True,padx=(8,0))
        self.split_tree=ttk.Treeview(right,columns=('split','images','labelled','nojson','boxes'),show='headings',height=4)
        for column,text,width in (('split','Tập',70),('images','Ảnh',80),('labelled','Ảnh có nhãn',110),('nojson','Ảnh không có .json',150),('boxes','Số khung',90)):
            self.split_tree.heading(column,text=text); self.split_tree.column(column,width=width,anchor='w' if column=='split' else 'e')
        self.split_tree.pack(fill='x')
        ttk.Label(right,text='Cảnh báo / lỗi').pack(anchor='w',pady=(8,2))
        self.data_messages=tk.Text(right,height=9,wrap='word',state='disabled',background='#fbfbfb')
        self.data_messages.tag_configure('error',foreground='#b91c1c'); self.data_messages.tag_configure('warn',foreground='#b45309')
        self.data_messages.tag_configure('ok',foreground='#047857'); self.data_messages.pack(fill='both',expand=True)
        bottom=ttk.Frame(tab); bottom.pack(fill='x',pady=(10,0))
        self.use_data_button=ttk.Button(bottom,text='Dùng dữ liệu này  →  sang bước Huấn luyện',style='Primary.TButton',command=self.use_data,state='disabled')
        self.use_data_button.pack(side='right')

    def choose_data(self):
        start=self.data_var.get() or str(Path.home())
        folder=filedialog.askdirectory(title='Chọn thư mục chứa train/ và val/',initialdir=start if Path(start).is_dir() else str(Path.home()))
        if folder: self.data_var.set(folder); self.scan_data()

    def scan_data(self):
        path=self.data_var.get().strip()
        if not path: messagebox.showinfo('Dữ liệu','Hãy chọn thư mục dữ liệu trước.'); return
        self.data_summary.set('Đang đọc dữ liệu...'); self.use_data_button.configure(state='disabled')
        self.background(lambda:scan(path),self.show_scan)

    def show_scan(self,report):
        if isinstance(report,Exception):
            self.data_summary.set('Lỗi khi đọc dữ liệu: '+str(report)); return
        self.report=report
        for tree in (self.label_tree,self.split_tree): tree.delete(*tree.get_children())
        for label in report['label_order']:
            self.label_tree.insert('', 'end',values=(label,report['labels'][label]['train'],report['labels'][label]['val']))
        for split,info in report['splits'].items():
            self.split_tree.insert('', 'end',values=(split,info['images'],info['labelled'],info['without_json'],info['boxes']))
        box=self.data_messages; box.configure(state='normal'); box.delete('1.0','end')
        for message in report['errors']: box.insert('end','LỖI: '+message+'\n','error')
        for message in report['warnings']: box.insert('end','Cảnh báo: '+message+'\n','warn')
        if not report['errors'] and not report['warnings']: box.insert('end','Không có cảnh báo.\n','ok')
        box.configure(state='disabled')
        if report['ok']:
            train,val=report['splits']['train'],report['splits']['val']
            self.data_summary.set(f'Hợp lệ: {report["path"]}\ntrain {train["images"]} ảnh / {train["boxes"]} khung   |   val {val["images"]} ảnh / {val["boxes"]} khung   |   {len(report["labels"])} nhãn')
            self.use_data_button.configure(state='normal'); self.settings['data_dir']=report['path']; self.save_settings()
        else: self.data_summary.set('Dữ liệu chưa hợp lệ. Xem phần lỗi bên phải.')

    def use_data(self):
        if not self.class_text.get('1.0','end').strip(): self.fill_classes()
        self.notebook.select(self.tab_train)

    # ------------------------------------------------------------ TAB 2: huấn luyện
    def build_train_tab(self):
        tab=self.tab_train; saved=self.settings.get('train',{})
        left=ttk.Frame(tab); left.pack(side='left',fill='y'); right=ttk.Frame(tab); right.pack(side='left',fill='both',expand=True,padx=(14,0))
        ttk.Label(left,text='Bước 2 - Tham số huấn luyện',style='Title.TLabel').pack(anchor='w')
        form=ttk.LabelFrame(left,text='Cơ bản',padding=8); form.pack(fill='x',pady=(6,0))
        ttk.Label(form,text='Tên nhãn (mỗi dòng một nhãn, đúng như trong LabelMe).\nThứ tự dòng = id của lớp: dòng đầu là id 0.').grid(row=0,column=0,columnspan=3,sticky='w')
        self.class_text=tk.Text(form,width=36,height=6); self.class_text.grid(row=1,column=0,columnspan=2,sticky='we',pady=4)
        self.class_text.insert('1.0','\n'.join(saved.get('class_names',[])))
        ttk.Button(form,text='Lấy từ\ndữ liệu',command=self.fill_classes).grid(row=1,column=2,sticky='n',padx=(6,0),pady=4)
        def field(row,text,widget,hint=''):
            ttk.Label(form,text=text).grid(row=row,column=0,sticky='w',pady=3); widget.grid(row=row,column=1,sticky='we',pady=3)
            if hint: ttk.Label(form,text=hint,style='Hint.TLabel').grid(row=row,column=2,sticky='w',padx=(6,0))
        self.epochs_var=tk.StringVar(value=str(saved.get('epochs',100))); field(2,'Epoch',ttk.Spinbox(form,from_=1,to=5000,textvariable=self.epochs_var,width=12),'số vòng lặp')
        self.batch_var=tk.StringVar(value=str(saved.get('batch',16)))
        field(3,'Batch',ttk.Combobox(form,textvariable=self.batch_var,values=['-1','4','8','16','32','64'],width=12),'-1 = tự chọn')
        self.imgsz_var=tk.StringVar(value=str(saved.get('imgsz',640)))
        field(4,'imgsz',ttk.Combobox(form,textvariable=self.imgsz_var,values=['320','416','512','640','800','960','1280'],width=12),'bội số của 32')
        self.model_var=tk.StringVar(value=saved.get('base_model','yolo11n.pt')); self.model_box=ttk.Combobox(form,textvariable=self.model_var,width=12,state='readonly')
        field(5,'Mô hình gốc',self.model_box,'n nhanh nhất, l chính xác nhất')
        self.device_var=tk.StringVar(value=saved.get('device','CPU' if os.name=='nt' else 'GPU 0')); self.device_box=ttk.Combobox(form,textvariable=self.device_var,width=12,state='readonly')
        field(6,'Thiết bị',self.device_box)
        self.name_var=tk.StringVar(value=saved.get('run_name','train')); field(7,'Tên lần train',ttk.Entry(form,textvariable=self.name_var,width=14),'chữ, số, _ -')
        form.columnconfigure(1,weight=1)
        advanced=ttk.LabelFrame(left,text='Nâng cao (có thể để mặc định)',padding=8); advanced.pack(fill='x',pady=(8,0))
        self.patience_var=tk.StringVar(value=str(saved.get('patience',50))); self.workers_var=tk.StringVar(value=str(saved.get('workers',8)))
        self.lr_var=tk.StringVar(value=str(saved.get('lr0',0.01))); self.optimizer_var=tk.StringVar(value=saved.get('optimizer','auto'))
        self.cos_var=tk.BooleanVar(value=saved.get('cos_lr',False)); self.cache_var=tk.BooleanVar(value=saved.get('cache',False))
        self.background_var=tk.BooleanVar(value=saved.get('include_background',True))
        rows=(('Patience (dừng sớm nếu không tiến bộ sau N epoch)',ttk.Spinbox(advanced,from_=0,to=1000,textvariable=self.patience_var,width=8)),
              ('Số luồng đọc dữ liệu (workers)',ttk.Spinbox(advanced,from_=0,to=32,textvariable=self.workers_var,width=8)),
              ('Learning rate ban đầu (lr0)',ttk.Entry(advanced,textvariable=self.lr_var,width=10)),
              ('Optimizer',ttk.Combobox(advanced,textvariable=self.optimizer_var,values=['auto','SGD','Adam','AdamW'],width=8,state='readonly')))
        for index,(text,widget) in enumerate(rows):
            ttk.Label(advanced,text=text).grid(row=index,column=0,sticky='w',pady=2); widget.grid(row=index,column=1,sticky='e',pady=2,padx=(8,0))
        ttk.Checkbutton(advanced,text='Giảm learning rate theo cosine',variable=self.cos_var).grid(row=4,column=0,columnspan=2,sticky='w')
        ttk.Checkbutton(advanced,text='Nạp ảnh vào RAM (nhanh hơn, tốn RAM)',variable=self.cache_var).grid(row=5,column=0,columnspan=2,sticky='w')
        ttk.Checkbutton(advanced,text='Dùng ảnh không có nhãn làm ảnh nền',variable=self.background_var).grid(row=6,column=0,columnspan=2,sticky='w')
        buttons=ttk.Frame(left); buttons.pack(fill='x',pady=(12,0))
        self.start_button=ttk.Button(buttons,text='▶  Bắt đầu train',style='Primary.TButton',command=self.start_training); self.start_button.pack(side='left')
        self.stop_button=ttk.Button(buttons,text='■  Dừng',command=self.stop_training,state='disabled'); self.stop_button.pack(side='left',padx=6)
        ttk.Label(left,style='Hint.TLabel',wraplength=380,justify='left',text='Mỗi lần train được lưu trong Training_model/runs/<tên>. '
                  'Mô hình tốt nhất là weights/best.pt. Có thể dừng giữa chừng rồi "Train tiếp" ở tab Kết quả.').pack(anchor='w',pady=(10,0))

        ttk.Label(right,text='Tiến trình',style='Title.TLabel').pack(anchor='w')
        self.train_state=tk.StringVar(value='Chưa chạy.'); ttk.Label(right,textvariable=self.train_state,justify='left').pack(anchor='w',pady=(4,2))
        self.progress=ttk.Progressbar(right,maximum=100); self.progress.pack(fill='x')
        self.train_metrics=tk.StringVar(value=''); ttk.Label(right,textvariable=self.train_metrics,style='Hint.TLabel',justify='left').pack(anchor='w',pady=(4,6))
        ttk.Label(right,text='Nhật ký').pack(anchor='w')
        frame=ttk.Frame(right); frame.pack(fill='both',expand=True)
        self.log=tk.Text(frame,wrap='none',state='disabled',background='#0f172a',foreground='#e2e8f0',insertbackground='#e2e8f0',font=('TkFixedFont',9))
        scroll=ttk.Scrollbar(frame,command=self.log.yview); self.log.configure(yscrollcommand=scroll.set)
        scroll.pack(side='right',fill='y'); self.log.pack(side='left',fill='both',expand=True)
        self.log.mark_set('live','end-1c'); self.log.mark_gravity('live','left')

    def fill_classes(self):
        if not self.report or not self.report.get('label_order'):
            messagebox.showinfo('Tên nhãn','Hãy kiểm tra dữ liệu ở tab 1 trước.'); return
        self.class_text.delete('1.0','end'); self.class_text.insert('1.0','\n'.join(self.report['label_order']))

    def device_value(self):
        text=self.device_var.get()
        if text=='CPU': return 'cpu'
        return ','.join(re.findall(r'\d+',text)) or '0'

    def collect_config(self):
        names=[line.strip() for line in self.class_text.get('1.0','end').splitlines() if line.strip()]
        if not names: raise ValueError('Chưa nhập tên nhãn.')
        if len(set(names))!=len(names): raise ValueError('Tên nhãn bị trùng.')
        report=self.report
        if not report or report['path']!=str(Path(self.data_var.get().strip()).expanduser()) or not report['ok']:
            report=scan(self.data_var.get().strip()); self.report=report
        if not report['ok']: raise ValueError('Dữ liệu chưa hợp lệ: '+' '.join(report['errors'])+'\nKiểm tra lại ở tab 1.')
        unknown=[n for n in names if n not in report['labels']]
        if unknown: raise ValueError('Nhãn không có trong dữ liệu LabelMe: '+', '.join(unknown)+'\nTên phải giống hệt trong LabelMe (kể cả chữ hoa/thường).')
        try: epochs,batch,imgsz=int(self.epochs_var.get()),int(self.batch_var.get()),int(self.imgsz_var.get())
        except ValueError: raise ValueError('Epoch, batch và imgsz phải là số nguyên.')
        if not 1<=epochs<=5000: raise ValueError('Epoch phải từ 1 đến 5000.')
        if batch!=-1 and not 1<=batch<=512: raise ValueError('Batch phải từ 1 đến 512 (hoặc -1 để tự chọn).')
        if imgsz%32 or not 64<=imgsz<=1920: raise ValueError('imgsz phải là bội số của 32, từ 64 đến 1920.')
        try: lr0=float(self.lr_var.get()); patience=int(self.patience_var.get()); workers=int(self.workers_var.get())
        except ValueError: raise ValueError('Giá trị trong phần Nâng cao không hợp lệ.')
        if not 0<lr0<1: raise ValueError('lr0 phải nằm giữa 0 và 1.')
        base=self.model_var.get(); local=WEIGHTS/base
        return {'data_dir':report['path'],'class_names':names,'base_model':str(local) if local.is_file() else base,'epochs':epochs,'batch':batch,
            'imgsz':imgsz,'device':self.device_value(),'patience':patience,'workers':workers,'optimizer':self.optimizer_var.get(),'lr0':lr0,
            'cos_lr':self.cos_var.get(),'cache':self.cache_var.get(),'include_background':self.background_var.get(),'seed':0,
            'created':time.strftime('%Y-%m-%d %H:%M:%S')},report

    def start_training(self,resume_run=None):
        if self.training_running(): messagebox.showinfo('Huấn luyện','Đang có một lần train chạy.'); return
        if not deps_ready(): messagebox.showwarning('Huấn luyện','Chưa cài môi trường train. Bấm "Cài môi trường" ở phía trên trước.'); return
        if shutil.disk_usage(HERE).free<1e9:
            messagebox.showerror('Huấn luyện','Ổ đĩa còn dưới 1 GB trống. Hãy giải phóng dung lượng trước khi train.'); return
        try:
            if resume_run is not None:
                run=resume_run; config=read_json(run/'config.json'); last=run/'weights/last.pt'
                if not last.is_file(): raise ValueError('Lần train này chưa có weights/last.pt để chạy tiếp.')
                config['resume_from']=str(last)
            else:
                config,report=self.collect_config()
                lost=[label for label in report['labels'] if label not in config['class_names']]
                if lost and not messagebox.askyesno('Tên nhãn','Các nhãn sau có trong dữ liệu nhưng không nằm trong danh sách và sẽ bị bỏ qua:\n'
                                                    +', '.join(lost)+'\n\nVẫn train?'): return
                stem=SAFE_NAME.sub('_',self.name_var.get().strip()) or 'train'; name=stem; counter=2
                while (RUNS/name).exists(): name=f'{stem}_{counter}'; counter+=1
                run=RUNS/name; run.mkdir(parents=True); config['run_dir']=str(run)
                self.settings['train']={k:config[k] for k in ('class_names','epochs','batch','imgsz','patience','workers','optimizer','lr0','cos_lr','cache','include_background')}
                self.settings['train'].update(base_model=self.model_var.get(),device=self.device_var.get(),run_name=self.name_var.get().strip()); self.save_settings()
        except ValueError as error:
            messagebox.showerror('Huấn luyện',str(error)); return
        (run/'config.json').write_text(json.dumps(config,ensure_ascii=False,indent=2),encoding='utf-8')
        (run/'status.json').write_text(json.dumps({'state':'starting','time':time.time()}),encoding='utf-8')
        handle=(run/'train.log').open('ab')
        self.process=subprocess.Popen(ml_command(HERE/'train_worker.py',run/'config.json'),stdout=handle,stderr=subprocess.STDOUT,
                                      cwd=str(HERE),env=worker_environment(),**process_options())
        self.active_run=run; self.log_offset=(run/'train.log').stat().st_size if resume_run is not None else 0; self.log_tail=''; self.train_started=time.time()
        self.log_clear(); self.log_append(f'== {run.name} ==\n')
        self.start_button.configure(state='disabled'); self.stop_button.configure(state='normal')
        self.notebook.select(self.tab_train); self.refresh_runs(select=run.name)

    def stop_training(self):
        if not self.training_running(): return
        if not messagebox.askyesno('Dừng train','Dừng lần train đang chạy?\nCó thể "Train tiếp" sau ở tab Kết quả (từ epoch đã lưu gần nhất).'): return
        self.kill_training()

    def kill_training(self):
        process=self.process
        if process is None or process.poll() is not None: return
        stop_process(process)
        if self.active_run is not None:
            (self.active_run/'status.json').write_text(json.dumps({'state':'stopped','time':time.time()}),encoding='utf-8')

    def log_clear(self):
        self.log.configure(state='normal'); self.log.delete('1.0','end'); self.log.mark_set('live','end-1c'); self.log.configure(state='disabled')

    def log_append(self,chunk):
        """Thêm văn bản vào nhật ký; dòng tiến độ (kết thúc bằng \\r) được ghi đè tại chỗ."""
        text=self.log_tail+ANSI.sub('',chunk); parts=text.split('\n'); self.log_tail=parts.pop()
        def visible(line):
            segments=[s for s in line.split('\r') if s.strip()]
            return segments[-1] if segments else ''
        self.log.configure(state='normal'); self.log.delete('live','end-1c')
        for line in parts: self.log.insert('end',visible(line)+'\n')
        self.log.mark_set('live','end-1c'); self.log.insert('end',visible(self.log_tail))
        lines=int(self.log.index('end-1c').split('.')[0])
        if lines>3000: self.log.delete('1.0',f'{lines-3000}.0')
        self.log.see('end'); self.log.configure(state='disabled')

    def poll_log(self,path,offset):
        if not path.is_file(): return offset
        size=path.stat().st_size
        if size<=offset: return offset
        with path.open('rb') as handle:
            handle.seek(max(offset,size-300_000)); data=handle.read()
        self.log_append(data.decode('utf-8','replace')); return size

    def update_training_view(self):
        run=self.active_run
        if run is None: return
        self.log_offset=self.poll_log(run/'train.log',self.log_offset)
        summary=run_summary(run,run.name if self.training_running() else None)
        total=summary['epochs'] or 0; done=summary['epochs_done']
        self.progress.configure(maximum=max(1,total),value=min(done,total))
        elapsed=time.time()-self.train_started if self.train_started else 0
        eta=''
        if self.training_running() and done>0 and total>done:
            eta=f'   |   còn khoảng {time.strftime("%H:%M:%S",time.gmtime(elapsed/done*(total-done)))}'
        self.train_state.set(f'{run.name}: {STATE_TEXT.get(summary["state"],summary["state"])}   |   epoch {done}/{total}'
                             f'   |   đã chạy {time.strftime("%H:%M:%S",time.gmtime(elapsed))}{eta}')
        self.train_metrics.set(f'Epoch gần nhất:  mAP50 {fmt(summary["map50"])}   mAP50-95 {fmt(summary["map50_95"])}   '
                               f'precision {fmt(summary["precision"])}   recall {fmt(summary["recall"])}')
        if not self.training_running() and str(self.stop_button['state'])=='normal':
            self.start_button.configure(state='normal'); self.stop_button.configure(state='disabled'); self.refresh_runs(select=run.name)
            status=read_json(run/'status.json')
            if status.get('state') in ('starting','preparing','training'):
                (run/'status.json').write_text(json.dumps({'state':'failed','error':'Tiến trình train thoát bất thường. Xem nhật ký.','time':time.time()}),encoding='utf-8')
            if status.get('state')=='failed': messagebox.showerror('Huấn luyện','Train bị lỗi:\n'+str(status.get('error',''))+'\n\nXem chi tiết trong khung Nhật ký.')

    # ------------------------------------------------------------ TAB 3: biểu đồ
    def build_chart_tab(self):
        tab=self.tab_chart; top=ttk.Frame(tab); top.pack(fill='x')
        ttk.Label(top,text='Lần train:').pack(side='left')
        self.chart_run=tk.StringVar(); self.chart_box=ttk.Combobox(top,textvariable=self.chart_run,state='readonly',width=34); self.chart_box.pack(side='left',padx=6)
        self.chart_box.bind('<<ComboboxSelected>>',lambda e:self.draw_charts())
        ttk.Button(top,text='Làm mới',command=self.draw_charts).pack(side='left')
        self.chart_auto=tk.BooleanVar(value=True); ttk.Checkbutton(top,text='Tự cập nhật khi đang train',variable=self.chart_auto).pack(side='left',padx=12)
        self.chart_info=tk.StringVar(); ttk.Label(top,textvariable=self.chart_info,style='Hint.TLabel').pack(side='left',padx=12)
        grid=ttk.Frame(tab); grid.pack(fill='both',expand=True,pady=(8,0))
        self.charts=[]
        for index in range(4):
            canvas=tk.Canvas(grid,background='#f8fafc',highlightthickness=1,highlightbackground='#d1d5db')
            canvas.grid(row=index//2,column=index%2,sticky='nsew',padx=4,pady=4); canvas.bind('<Configure>',lambda e:self.after_idle(self.draw_charts))
            self.charts.append(canvas)
        for k in range(2): grid.columnconfigure(k,weight=1); grid.rowconfigure(k,weight=1)
        ttk.Label(tab,style='Hint.TLabel',text='Loss train giảm đều là bình thường. Nếu loss val tăng trở lại trong khi loss train vẫn giảm thì mô hình đang học thuộc (overfit): '
                  'cần thêm ảnh hoặc dừng sớm. mAP50-95 là thước đo chính của độ chính xác.').pack(anchor='w',pady=(6,0))

    def draw_charts(self):
        name=self.chart_run.get()
        if not name or not (RUNS/name).is_dir():
            for canvas,title in zip(self.charts,('Loss (train)','Loss (val)','Độ chính xác (val)','Learning rate')): draw_chart(canvas,title,{})
            self.chart_info.set(''); return
        columns=read_results(RUNS/name)
        pick=lambda prefix:{k.split('/',1)[1]:v for k,v in columns.items() if k.startswith(prefix)}
        metrics={k.replace('metrics/','').replace('(B)',''):v for k,v in columns.items() if k.startswith('metrics/')}
        draw_chart(self.charts[0],'Loss (train)',pick('train/')); draw_chart(self.charts[1],'Loss (val)',pick('val/'))
        draw_chart(self.charts[2],'Độ chính xác trên tập val',metrics); draw_chart(self.charts[3],'Learning rate',pick('lr/'))
        best=None; values=next((v for k,v in columns.items() if k.startswith('metrics/mAP50-95')),[])
        usable=[(v,i) for i,v in enumerate(values) if v is not None]
        if usable: best=max(usable)
        self.chart_info.set(f'{len(columns.get("epoch",[]))} epoch'+(f'   |   mAP50-95 tốt nhất {best[0]:.4f} ở epoch {best[1]+1}' if best else ''))

    # ------------------------------------------------------------ TAB 4: kiểm tra mô hình
    def build_test_tab(self):
        tab=self.tab_test; top=ttk.Frame(tab); top.pack(fill='x')
        ttk.Label(top,text='Mô hình:').pack(side='left')
        self.test_model=tk.StringVar(); self.test_model_box=ttk.Combobox(top,textvariable=self.test_model,state='readonly',width=40); self.test_model_box.pack(side='left',padx=6)
        self.test_model_box.bind('<<ComboboxSelected>>',lambda e:self.run_predict())
        ttk.Label(top,text='Ngưỡng tin cậy:').pack(side='left',padx=(10,0))
        self.conf_var=tk.DoubleVar(value=self.settings.get('conf',0.25)); self.conf_label=tk.StringVar(value=f'{self.conf_var.get():.2f}')
        scale=ttk.Scale(top,from_=0.05,to=0.95,variable=self.conf_var,length=150,command=lambda v:self.conf_label.set(f'{float(v):.2f}')); scale.pack(side='left',padx=4)
        scale.bind('<ButtonRelease-1>',lambda e:self.run_predict()); ttk.Label(top,textvariable=self.conf_label,width=5).pack(side='left')
        ttk.Button(top,text='Thử ngưỡng 0.05',command=self.test_low_confidence).pack(side='left',padx=4)
        ttk.Label(top,text='imgsz:').pack(side='left',padx=(10,0)); self.test_imgsz=tk.StringVar(value='640')
        box=ttk.Combobox(top,textvariable=self.test_imgsz,values=['320','416','512','640','800','960','1280'],width=6); box.pack(side='left',padx=4)
        box.bind('<<ComboboxSelected>>',lambda e:self.run_predict())
        ttk.Label(top,text='Thiết bị:').pack(side='left',padx=(10,0)); self.test_device=tk.StringVar(value='GPU 0')
        self.test_device_box=ttk.Combobox(top,textvariable=self.test_device,state='readonly',width=7); self.test_device_box.pack(side='left',padx=4)
        row=ttk.Frame(tab); row.pack(fill='x',pady=(8,6))
        ttk.Button(row,text='Mở ảnh...',style='Primary.TButton',command=self.open_test_image).pack(side='left')
        ttk.Button(row,text='Mở thư mục ảnh...',command=self.open_test_folder).pack(side='left',padx=6)
        ttk.Button(row,text='◀ Ảnh trước',command=lambda:self.step_test(-1)).pack(side='left'); ttk.Button(row,text='Ảnh sau ▶',command=lambda:self.step_test(1)).pack(side='left',padx=6)
        ttk.Button(row,text='Lưu ảnh kết quả...',command=self.save_test_result).pack(side='left')
        self.test_info=tk.StringVar(value='Chọn mô hình rồi mở một ảnh để xem kết quả phát hiện.'); ttk.Label(row,textvariable=self.test_info,style='Hint.TLabel').pack(side='left',padx=12)
        body=ttk.Frame(tab); body.pack(fill='both',expand=True)
        self.test_canvas=tk.Canvas(body,background='#1f2937',highlightthickness=0); self.test_canvas.pack(side='left',fill='both',expand=True)
        self.test_canvas.bind('<Configure>',lambda e:self.after_idle(self.render_test))
        side=ttk.Frame(body); side.pack(side='left',fill='y',padx=(8,0))
        ttk.Label(side,text='Các vật thể phát hiện được').pack(anchor='w')
        self.det_tree=ttk.Treeview(side,columns=('label','conf','box'),show='headings',height=22)
        for column,text,width in (('label','Nhãn',120),('conf','Tin cậy',70),('box','Khung (x0, y0, x1, y1)',190)):
            self.det_tree.heading(column,text=text); self.det_tree.column(column,width=width,anchor='w')
        self.det_tree.pack(fill='y',expand=True)
        self.det_count=tk.StringVar(); ttk.Label(side,textvariable=self.det_count,wraplength=370,justify='left').pack(anchor='w',pady=(6,0))

    def model_choices(self):
        choices={}
        for run in list_runs():
            for kind in ('best','last'):
                checkpoint=run/'weights'/f'{kind}.pt'
                if checkpoint.is_file():
                    choices[f'{run.name}  ({kind}.pt)']=checkpoint
        for weight in sorted(WEIGHTS.glob('*.pt')): choices[f'[gốc] {weight.name}  (COCO 80 lớp)']=weight
        return choices

    def open_test_image(self):
        start=self.settings.get('test_dir',str(Path.home()))
        path=filedialog.askopenfilename(title='Chọn ảnh',initialdir=start if Path(start).is_dir() else str(Path.home()),
                                        filetypes=[('Ảnh','*.jpg *.jpeg *.png *.bmp *.webp *.tif *.tiff'),('Tất cả','*')])
        if not path: return
        folder=Path(path).parent; self.settings['test_dir']=str(folder); self.save_settings()
        self.test_images=sorted(p for p in folder.iterdir() if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
        self.test_index=self.test_images.index(Path(path)) if Path(path) in self.test_images else 0
        if Path(path) not in self.test_images: self.test_images=[Path(path)]
        self.show_test_image()

    def open_test_folder(self):
        start=self.settings.get('test_dir',str(Path.home()))
        folder=filedialog.askdirectory(title='Chọn thư mục ảnh',initialdir=start if Path(start).is_dir() else str(Path.home()))
        if not folder: return
        images=sorted(p for p in Path(folder).iterdir() if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
        if not images: messagebox.showinfo('Kiểm tra','Thư mục này không có ảnh.'); return
        self.settings['test_dir']=folder; self.save_settings(); self.test_images=images; self.test_index=0; self.show_test_image()

    def step_test(self,delta):
        if not self.test_images: return
        self.test_index=(self.test_index+delta)%len(self.test_images); self.show_test_image()

    def show_test_image(self):
        from PIL import Image
        path=self.test_images[self.test_index]
        try: self.test_image=Image.open(path).convert('RGB')
        except Exception as error: messagebox.showerror('Kiểm tra',f'Không mở được ảnh:\n{error}'); return
        self.test_result=None; self.render_test(); self.run_predict()

    def render_test(self):
        canvas=self.test_canvas; canvas.delete('all')
        if self.test_image is None:
            canvas.create_text(canvas.winfo_width()/2,canvas.winfo_height()/2,text='Chưa mở ảnh',fill='#9ca3af'); return
        width,height=canvas.winfo_width(),canvas.winfo_height()
        self.test_photo,scale=photo_from_image(self.test_image,width-8,height-8)
        ox=(width-self.test_photo.width())/2; oy=(height-self.test_photo.height())/2
        canvas.create_image(ox,oy,image=self.test_photo,anchor='nw')
        if not self.test_result: return
        for detection in self.test_result['detections']:
            colour=COLOURS[detection['class_id']%len(COLOURS)]; x0,y0,x1,y1=[v*scale for v in detection['bbox_xyxy']]
            canvas.create_rectangle(ox+x0,oy+y0,ox+x1,oy+y1,outline=colour,width=2)
            label=f"{detection['class_name']} {detection['confidence']:.2f}"
            item=canvas.create_text(ox+x0+3,oy+y0-2,text=label,anchor='sw',fill='#ffffff',font=('TkDefaultFont',9,'bold'))
            box=canvas.bbox(item)
            if box[1]<0: canvas.move(item,0,box[3]-box[1]+4); box=canvas.bbox(item)
            canvas.tag_lower(canvas.create_rectangle(box[0]-3,box[1]-1,box[2]+3,box[3]+1,fill=colour,outline=''),item)

    def ensure_predict_worker(self):
        if self.predict_process is not None and self.predict_process.poll() is None: return
        self.predict_process=subprocess.Popen(ml_command(HERE/'predict_worker.py'),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                              stderr=subprocess.DEVNULL,text=True,encoding="utf-8",errors="replace",bufsize=1,cwd=str(HERE),env=worker_environment(),**process_options())
        process=self.predict_process
        def reader():
            for line in process.stdout:
                try: self.ui_queue.put((self.on_predict_reply,json.loads(line)))
                except ValueError: pass
            self.ui_queue.put((self.on_predict_reply,{'error':'Tiến trình chạy thử mô hình đã thoát.','fatal':True}))
        threading.Thread(target=reader,daemon=True).start()

    def test_low_confidence(self):
        self.conf_var.set(0.05)
        self.conf_label.set('0.05')
        self.run_predict()

    def run_predict(self):
        if self.test_image is None or not self.test_images: return
        choices=self.model_choices(); model=choices.get(self.test_model.get())
        if model is None: self.test_info.set('Chưa chọn mô hình.'); return
        if not deps_ready(): self.test_info.set('Chưa cài môi trường (bấm "Cài môi trường" ở phía trên).'); return
        if self.predict_busy: self.predict_pending=True; return
        try: imgsz=int(self.test_imgsz.get())
        except ValueError: imgsz=640
        self.ensure_predict_worker(); self.predict_busy=True; self.predict_pending=False
        self.settings['conf']=round(self.conf_var.get(),2); self.save_settings()
        self.test_info.set('Đang chạy mô hình... (lần đầu mất vài giây để nạp)')
        device='cpu' if self.test_device.get()=='CPU' else (re.findall(r'\d+',self.test_device.get()) or ['0'])[0]
        request={'id':str(self.test_images[self.test_index]),'model':str(model),'image':str(self.test_images[self.test_index]),
                 'conf':self.conf_var.get(),'imgsz':imgsz,'device':device}
        try: self.predict_process.stdin.write(json.dumps(request)+'\n'); self.predict_process.stdin.flush()
        except (BrokenPipeError,OSError): self.predict_busy=False; self.test_info.set('Không gửi được yêu cầu tới tiến trình chạy thử.')

    def on_predict_reply(self,reply):
        if reply.get('ready'): return
        self.predict_busy=False
        if reply.get('error'):
            self.test_info.set('Lỗi: '+reply['error'])
            if reply.get('fatal'): self.predict_process=None
            return
        if self.test_images and reply.get('id')==str(self.test_images[self.test_index]):
            self.test_result=reply; self.det_tree.delete(*self.det_tree.get_children())
            for d in sorted(reply['detections'],key=lambda d:-d['confidence']):
                self.det_tree.insert('', 'end',values=(d['class_name'],f"{d['confidence']:.3f}",', '.join(f'{v:.0f}' for v in d['bbox_xyxy'])))
            counts={}
            for d in reply['detections']: counts[d['class_name']]=counts.get(d['class_name'],0)+1
            self.det_count.set('Tổng: '+(', '.join(f'{n} {k}' for k,n in sorted(counts.items())) or 'không phát hiện vật thể nào'))
            self.test_info.set(f'{self.test_images[self.test_index].name}   ({self.test_index+1}/{len(self.test_images)})   |   '
                               f'{reply["width"]}x{reply["height"]}   |   {reply["time_ms"]:.0f} ms   |   {len(reply["detections"])} vật thể')
            self.render_test()
        if getattr(self,'predict_pending',False): self.run_predict()

    def save_test_result(self):
        if self.test_image is None or not self.test_result: messagebox.showinfo('Kiểm tra','Chưa có kết quả để lưu.'); return
        from PIL import ImageDraw
        target=filedialog.asksaveasfilename(title='Lưu ảnh kết quả',defaultextension='.png',initialfile=self.test_images[self.test_index].stem+'_detect.png',
                                            filetypes=[('PNG','*.png'),('JPEG','*.jpg')])
        if not target: return
        image=self.test_image.copy(); draw=ImageDraw.Draw(image)
        for d in self.test_result['detections']:
            colour=COLOURS[d['class_id']%len(COLOURS)]; x0,y0,x1,y1=d['bbox_xyxy']; label=f"{d['class_name']} {d['confidence']:.2f}"
            draw.rectangle([x0,y0,x1,y1],outline=colour,width=3); size=draw.textbbox((0,0),label)
            top=y0-size[3]-4 if y0>=size[3]+4 else y0
            draw.rectangle([x0,top,x0+size[2]+6,top+size[3]+4],fill=colour); draw.text((x0+3,top+2),label,fill='white')
        image.save(target); self.test_info.set('Đã lưu: '+target)

    # ------------------------------------------------------------ TAB 5: kết quả
    def build_runs_tab(self):
        tab=self.tab_runs; top=ttk.Frame(tab); top.pack(fill='x')
        ttk.Label(top,text='Các lần train (Training_model/runs)',style='Title.TLabel').pack(side='left')
        ttk.Button(top,text='Làm mới',command=self.refresh_runs).pack(side='right')
        columns=(('name','Tên',170),('state','Trạng thái',110),('epochs','Epoch',80),('map','mAP50-95',80),('map50','mAP50',70),('p','Precision',75),
                 ('r','Recall',70),('imgsz','imgsz',55),('model','Gốc',90),('classes','Nhãn',300),('onnx','ONNX',120))
        self.run_tree=ttk.Treeview(tab,columns=[c[0] for c in columns],show='headings',height=7,selectmode='browse')
        for column,text,width in columns:
            self.run_tree.heading(column,text=text); self.run_tree.column(column,width=width,anchor='w')
        self.run_tree.pack(fill='x',pady=(6,6)); self.run_tree.bind('<<TreeviewSelect>>',lambda e:self.on_run_selected())
        buttons=ttk.Frame(tab); buttons.pack(fill='x')
        ttk.Button(buttons,text='Xem biểu đồ loss',command=self.run_to_chart).pack(side='left')
        ttk.Button(buttons,text='Thử mô hình trên ảnh',command=self.run_to_test).pack(side='left',padx=6)
        ttk.Button(buttons,text='Train tiếp (resume)',command=self.resume_selected).pack(side='left')
        ttk.Label(buttons,text='   Xuất ONNX, imgsz:').pack(side='left'); self.export_imgsz=tk.StringVar(value='640')
        ttk.Combobox(buttons,textvariable=self.export_imgsz,values=['320','416','512','640','800','960','1280'],width=6).pack(side='left',padx=4)
        self.export_button=ttk.Button(buttons,text='Xuất ONNX cho robot',command=self.export_selected); self.export_button.pack(side='left')
        ttk.Button(buttons,text='Mở thư mục',command=self.open_run_folder).pack(side='left',padx=6)
        ttk.Button(buttons,text='Xoá lần train...',command=self.delete_selected).pack(side='right')
        self.run_info=tk.StringVar(); ttk.Label(tab,textvariable=self.run_info,style='Hint.TLabel',wraplength=1250,justify='left').pack(anchor='w',pady=(6,4))
        row=ttk.Frame(tab); row.pack(fill='x')
        ttk.Label(row,text='Ảnh đánh giá do Ultralytics tạo:').pack(side='left')
        self.plot_var=tk.StringVar(); self.plot_box=ttk.Combobox(row,textvariable=self.plot_var,state='readonly',width=44); self.plot_box.pack(side='left',padx=6)
        self.plot_box.bind('<<ComboboxSelected>>',lambda e:self.render_plot())
        self.plot_canvas=tk.Canvas(tab,background='#f3f4f6',highlightthickness=1,highlightbackground='#d1d5db'); self.plot_canvas.pack(fill='both',expand=True,pady=(6,0))
        self.plot_canvas.bind('<Configure>',lambda e:self.after_idle(self.render_plot))

    def selected_run(self):
        selection=self.run_tree.selection()
        if not selection: messagebox.showinfo('Kết quả','Hãy chọn một lần train trong bảng.'); return None
        run=RUNS/selection[0]
        return run if run.is_dir() else None

    def refresh_runs(self,select=None):
        active=self.active_run.name if (self.active_run is not None and self.training_running()) else None
        current=select or (self.run_tree.selection()[0] if self.run_tree.selection() else None)
        self.run_tree.delete(*self.run_tree.get_children()); names=[]
        for run in list_runs():
            s=run_summary(run,active); c=s['config']; names.append(run.name)
            self.run_tree.insert('', 'end',iid=run.name,values=(run.name,STATE_TEXT.get(s['state'],s['state']),f'{s["epochs_done"]}/{s["epochs"] or "?"}',
                fmt(s['map50_95']),fmt(s['map50']),fmt(s['precision']),fmt(s['recall']),c.get('imgsz','-'),Path(str(c.get('base_model',''))).name,
                ', '.join(c.get('class_names',[])),', '.join(s['onnx']) or '-'))
        if current in names: self.run_tree.selection_set(current)
        self.chart_box.configure(values=names)
        if select: self.chart_run.set(select)
        elif self.chart_run.get() not in names: self.chart_run.set(names[0] if names else '')
        choices=list(self.model_choices()); self.test_model_box.configure(values=choices)
        if self.test_model.get() not in choices: self.test_model.set(choices[0] if choices else '')
        models=sorted({p.name for p in WEIGHTS.glob('*.pt')}|set(DOWNLOADABLE)); self.model_box.configure(values=models)
        if self.model_var.get() not in models: self.model_var.set('yolo11n.pt')

    def on_run_selected(self):
        selection=self.run_tree.selection()
        if not selection: return
        run=RUNS/selection[0]
        if not run.is_dir(): return
        config=read_json(run/'config.json'); stats=read_json(run/'dataset_stats.json'); pieces=[f'Dữ liệu: {config.get("data_dir","-")}']
        for split,info in stats.get('splits',{}).items(): pieces.append(f'{split}: {info["images"]} ảnh, {info["boxes"]} khung')
        pieces.append(f'batch {config.get("batch","-")}, thiết bị {config.get("device","-")}, tạo lúc {config.get("created","-")}')
        error=read_json(run/'status.json').get('error')
        if error: pieces.append('LỖI: '+str(error))
        self.run_info.set('   |   '.join(pieces))
        order=['results.png','confusion_matrix_normalized.png','confusion_matrix.png','BoxPR_curve.png','BoxF1_curve.png','labels.jpg','val_batch0_pred.jpg','val_batch0_labels.jpg','train_batch0.jpg']
        files=sorted((p.name for p in run.iterdir() if p.suffix.lower() in ('.png','.jpg')),key=lambda n:(order.index(n) if n in order else len(order),n))
        self.plot_box.configure(values=files); self.plot_var.set(files[0] if files else ''); self.render_plot()

    def render_plot(self):
        canvas=self.plot_canvas; canvas.delete('all'); selection=self.run_tree.selection(); name=self.plot_var.get()
        if not selection or not name or not (RUNS/selection[0]/name).is_file():
            canvas.create_text(canvas.winfo_width()/2,canvas.winfo_height()/2,fill='#6b7280',
                text='Chọn một lần train. Ảnh đánh giá (ma trận nhầm lẫn, đường PR, ảnh dự đoán trên tập val...) xuất hiện sau khi train xong.'); return
        from PIL import Image
        try: image=Image.open(RUNS/selection[0]/name).convert('RGB')
        except Exception: return
        self.plot_photo,_=photo_from_image(image,canvas.winfo_width()-8,canvas.winfo_height()-8)
        canvas.create_image(canvas.winfo_width()/2,canvas.winfo_height()/2,image=self.plot_photo,anchor='center')

    def run_to_chart(self):
        run=self.selected_run()
        if run: self.chart_run.set(run.name); self.notebook.select(self.tab_chart); self.draw_charts()

    def run_to_test(self):
        run=self.selected_run()
        if not run: return
        match=[text for text,path in self.model_choices().items() if path.parent.parent==run]
        if not match: messagebox.showinfo('Kết quả','Lần train này chưa có file trọng số (best.pt / last.pt).'); return
        self.test_model.set(match[0]); self.notebook.select(self.tab_test); self.run_predict()

    def resume_selected(self):
        run=self.selected_run()
        if run and messagebox.askyesno('Train tiếp',f'Chạy tiếp "{run.name}" từ epoch đã lưu gần nhất (weights/last.pt)?'): self.start_training(resume_run=run)

    def export_selected(self):
        run=self.selected_run()
        if not run: return
        if not (run/'weights/best.pt').is_file(): messagebox.showinfo('Xuất ONNX','Lần train này chưa có best.pt.'); return
        try: imgsz=int(self.export_imgsz.get()); assert imgsz%32==0 and 64<=imgsz<=1920
        except (ValueError,AssertionError): messagebox.showerror('Xuất ONNX','imgsz phải là bội số của 32.'); return
        self.export_button.configure(state='disabled'); self.status_var.set(f'Đang xuất ONNX cho {run.name}...')
        def work():
            result=subprocess.run(ml_command(HERE/'export_worker.py',run,imgsz),capture_output=True,text=True,encoding="utf-8",errors="replace",cwd=str(HERE),env=worker_environment(),**process_options())
            line=next((l for l in result.stdout.splitlines() if l.startswith('EXPORT_OK ')),None)
            if line is None: raise RuntimeError((result.stdout+result.stderr)[-1500:])
            return json.loads(line[10:])
        def done(result):
            self.export_button.configure(state='normal'); self.refresh_runs()
            if isinstance(result,Exception): messagebox.showerror('Xuất ONNX','Xuất thất bại:\n'+str(result)); return
            messagebox.showinfo('Xuất ONNX','Đã xuất:\n'+result['onnx']+'\n\nTên lớp: '+result['class_names']+'\nsha256: '+result['sha256']+
                '\n\nĐể robot mô phỏng dùng mô hình này:\n- '+'\n- '.join(result['cach_dung_cho_robot_mo_phong'])+
                '\n\n(Hướng dẫn này cũng được lưu cạnh file .onnx.)')
        self.background(work,done)

    def open_run_folder(self):
        run=self.selected_run()
        if run: subprocess.Popen(['xdg-open',str(run)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)

    def delete_selected(self):
        run=self.selected_run()
        if not run: return
        if self.training_running() and self.active_run==run: messagebox.showinfo('Xoá','Lần train này đang chạy. Hãy dừng trước.'); return
        if not messagebox.askyesno('Xoá lần train',f'Xoá vĩnh viễn thư mục:\n{run}\n\ngồm cả mô hình đã train? Không thể hoàn tác.',icon='warning'): return
        shutil.rmtree(run,ignore_errors=True)
        if self.active_run==run: self.active_run=None
        self.refresh_runs(); self.run_info.set(''); self.render_plot()

    # ------------------------------------------------------------ vòng lặp nền
    def on_tab_changed(self):
        tab=self.notebook.select()
        if tab==str(self.tab_chart): self.draw_charts()
        elif tab==str(self.tab_runs): self.refresh_runs()

    def refresh_system(self):
        def work(): return gpu_info(),shutil.disk_usage(HERE),cuda_available()
        def done(result):
            if not isinstance(result,Exception):
                gpus,disk,can_cuda=result
                devices=([f'GPU {g[0]}' for g in gpus]+(['GPU '+' + '.join(g[0] for g in gpus)] if len(gpus)>1 else []) if can_cuda else [])+['CPU']
                self.device_box.configure(values=devices); self.test_device_box.configure(values=([f'GPU {g[0]}' for g in gpus] if can_cuda else [])+['CPU'])
                if self.device_var.get() not in devices: self.device_var.set(devices[0])
                if self.test_device.get() not in self.test_device_box['values']: self.test_device.set(self.test_device_box['values'][0])
                text='   |   '.join(f'GPU {g[0]} {g[1].replace("NVIDIA GeForce ","")}: {float(g[2])/1024:.1f}/{float(g[3])/1024:.0f} GB, {g[4]}%' for g in gpus) or 'Không thấy GPU (nvidia-smi)'
                free=disk.free/1e9
                self.status_var.set(text+f'   |   Ổ đĩa trống: {free:.1f} GB'+('  ⚠ sắp đầy' if free<5 else '')+f'   |   {HERE}')
            self.after(5000,self.refresh_system)
        self.background(work,done)

    def tick(self):
        if self.active_run is not None: self.update_training_view()
        if self.setup_process is not None:
            self.setup_offset=self.poll_log(self.setup_log,self.setup_offset)
            if self.setup_process.poll() is not None:
                code=self.setup_process.returncode; self.setup_process=None; self.build_banner(); self.refresh_runs()
                self.log_append('\n== Cài xong ==\n' if code==0 else f'\n== Cài thất bại (mã {code}) ==\n')
        if self.training_running() and self.chart_auto.get() and self.notebook.select()==str(self.tab_chart) and self.active_run is not None:
            if self.chart_run.get()==self.active_run.name and int(time.time())%3==0: self.draw_charts()
        self.after(1000,self.tick)

    def on_close(self):
        if self.training_running():
            if not messagebox.askyesno('Thoát','Đang train. Thoát sẽ DỪNG lần train này (có thể "Train tiếp" sau).\n\nThoát?'): return
            self.kill_training()
        if self.predict_process is not None and self.predict_process.poll() is None:
            stop_process(self.predict_process)
        self.destroy()


if __name__=='__main__':
    App().mainloop()

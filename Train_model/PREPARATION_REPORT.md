# Preparation report

Created under D:\unitree_g1\Train_model. No previous directory existed. All existing controllers, configs and navigation files were SHA256 compared before/after and unchanged. No installation, training, model replacement, commit or push.

Available: native GUI, image/camera import, video sampling, class editing with mapping protection after annotation, canvas box creation/deletion, explicit background confirmation, reviewed status, source/session grouping, content hash deduplication, atomic JSON saves with backups, audit, deterministic group-level train/val/test split, YOLO normalized labels, nano architecture definition with random initialization, class map and training plan snapshots. TRAIN button locked; there is no training execution code.

Ground-truth label proposals: tested against an existing Factory V2 camera RGB/metadata capture, produced six clipped valid boxes. These proposals use manifest assembly world AABBs and assume unchanged world alignment; they lack occlusion handling and must be manually reviewed. They are not AI outputs and do not modify the runtime detector.

Tests passed: import and duplicate rejection, invalid bbox rejection, group separation, label/config export, pretrained=false and training_started=false, no weight file creation, actual Tk/Pillow GUI initialization/render. Test images were temporary fixtures, not training data. An initial GUI test exposed a fixed image-root path error; final rerun passed. Console Unicode display needed Python -X utf8 for the Vietnamese audit; GUI uses native Unicode strings.

Current actual dataset: empty. Do not mistake successful tool tests for a prepared, labeled training dataset. Default classes are placeholders for user editing. plans/readiness.json records missing data and WAITING state. No dataset was silently marked background or automatically approved. GPU training backend has NOT been installed/validated: current separate YOLO interpreter has CPU-only PyTorch. Future GPU preparation and training require the user's decision. ONNX export/evaluation follow real training; no fake model was generated.

GUI launched with existing pythonw.exe for the user. Open again through open_app.bat. User next steps: choose classes, import real images by capture group, annotate/review, audit, then prepare snapshot. Stop at training_plan.json and decide when to proceed.

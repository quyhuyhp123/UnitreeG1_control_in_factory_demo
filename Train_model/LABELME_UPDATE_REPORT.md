# Labelme workflow update

Changed app.py to read-only training-preparation GUI. Removed annotation canvas bindings/buttons, raw-image/video import and Factory V2 proposals from active UI. Added labelme_import.py with strict rectangle validation, embedded image fallback, source archives/hashes, update-in-same-group and cross-group duplicate protection. Kept existing datasets and exports untouched. Updated tests and README.

Final test_preparation.py passed Labelme source preservation, import/dedup, invalid bbox/polygon rejection, empty-label confirmation, group splitting, YOLO export, random-init plan and actual Tk image/box preview. No training or package installation occurred. Existing CPU-only backend remains unchanged. Old app process may still show the old code until closed and reopened. Source image/JSON remain unchanged.

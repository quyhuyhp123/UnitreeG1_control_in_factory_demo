"""Windows port checks; fixture conversion and dummy process, NEVER training."""
import json,os,sys,tempfile,subprocess,unittest
from pathlib import Path
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'Training_model'))
from platform_runtime import ml_command,worker_environment,process_options,stop_process
from labelme_to_yolo import convert

@unittest.skipUnless(os.name=='nt','Native Windows checks')
class WindowsCompatibility(unittest.TestCase):
    def test_worker_command_and_stop(self):
        command=ml_command('-c','import time;time.sleep(30)')
        self.assertNotIn('bash',command)
        process=subprocess.Popen(command,env=worker_environment(),**process_options())
        try:stop_process(process);self.assertIsNotNone(process.poll())
        finally:
            if process.poll() is None:process.kill()

    def test_labelme_conversion_without_symlink_permission(self):
        with tempfile.TemporaryDirectory(dir=ROOT/'logs') as folder:
            base=Path(folder)
            for split in ['train','val']:
                source=base/'input'/split;source.mkdir(parents=True)
                Image.new('RGB',(100,80),'blue').save(source/'sample.png')
                (source/'sample.json').write_text(json.dumps({'imagePath':'sample.png','imageWidth':100,'imageHeight':80,
                    'shapes':[{'shape_type':'rectangle','label':'person','points':[[10,10],[80,70]]}]}))
            result=convert(base/'input',base/'output',['person'])
            self.assertEqual(result['splits']['train']['boxes'],1)
            image=next((base/'output/images/train').glob('*.png'))
            self.assertTrue(image.is_file());self.assertFalse(image.is_symlink())
            self.assertTrue((base/'input/train/sample.json').is_file())

if __name__=='__main__':unittest.main()

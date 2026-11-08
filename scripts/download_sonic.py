"""Pinned minimal official SONIC deployment download; no installs or training data.

Uses stdlib because huggingface_hub is absent from the verified Isaac Python.
The upstream download_from_hf.py file list was audited at the pinned commit.
Existing files are verified, never overwritten on hash mismatch.
"""
from pathlib import Path
import hashlib,json,urllib.request
ROOT=Path(r'D:\unitree_g1')


def sha256(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): h.update(block)
    return h.hexdigest()


def main():
    cfg=json.loads((ROOT/'configs/sonic.yaml').read_text())
    rev=cfg['hf_revision']
    base='https://huggingface.co'
    with urllib.request.urlopen(f'{base}/api/models/nvidia/GEAR-SONIC/tree/{rev}?recursive=true') as r:
        metadata={x['path']:x for x in json.load(r)}
    directory=ROOT/cfg['directory']; directory.mkdir(parents=True,exist_ok=True)
    result={'repository':'nvidia/GEAR-SONIC','revision':rev,'files':[]}
    for name in ['config.json','observation_config.yaml','model_encoder.onnx','model_decoder.onnx','planner_sonic.onnx']:
        info=metadata[name]; path=directory/name
        url=f'{base}/nvidia/GEAR-SONIC/resolve/{rev}/{name}'
        if not path.exists():
            temporary=path.with_suffix(path.suffix+'.part')
            if temporary.exists(): raise RuntimeError(f'Partial download exists; inspect before retry: {temporary}')
            urllib.request.urlretrieve(url,temporary)
            if temporary.stat().st_size!=info['size']: raise ValueError(f'Incomplete download: {name}')
            expected=info.get('lfs',{}).get('oid')
            if expected and sha256(temporary)!=expected: raise ValueError(f'Hash mismatch: {name}')
            temporary.rename(path)
        digest=sha256(path); expected=info.get('lfs',{}).get('oid')
        if path.stat().st_size!=info['size'] or (expected and digest!=expected):
            raise ValueError(f'Existing file differs from pinned release; preserved: {path}')
        result['files'].append({'name':name,'bytes':path.stat().st_size,'sha256':digest,
                                'upstream_lfs_sha256':expected,'url':url})
        print('Verified',name,path.stat().st_size,flush=True)
    (directory/'provenance.json').write_text(json.dumps(result,indent=2),encoding='utf-8')


if __name__=='__main__': main()

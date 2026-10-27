#!/usr/bin/env python3
"""Snapshot the package and pinned dependency sources for an IPC source build."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import shutil
import tarfile

PKG=Path(__file__).resolve().parents[1]
EXCLUDED_DIRS={'.git','__pycache__','build','devel','install','.pytest_cache','roslogs'}
EXCLUDED_SUFFIXES={'.pyc','.pyo','.log','.o','.a','.so','.deb'}


def copy_tree(source, target):
    for p in sorted(source.rglob('*')):
        rel=p.relative_to(source)
        if any(part in EXCLUDED_DIRS for part in rel.parts) or p.suffix in EXCLUDED_SUFFIXES:
            continue
        if p.is_symlink():
            raise RuntimeError('Refusing to package an unexpected symlink: '+str(p))
        if not p.is_file():
            continue
        out=target/rel
        out.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,out)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--name',required=True)
    p.add_argument('--deps',type=Path,required=True)
    p.add_argument('--source-versions',type=Path,required=True)
    p.add_argument('--readme',type=Path,default=PKG/'IPC_BUNDLE_CN.md')
    a=p.parse_args()
    if Path(a.name).name!=a.name or a.name in ('','.','..'):
        p.error('--name must be one directory name')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    stage=a.output_dir/a.name
    archive=a.output_dir/(a.name+'.tar.gz')
    if stage.exists() or archive.exists():
        p.error('Output exists; choose another --name')
    for name in ('osqp','osqp-eigen','qdldl'):
        if not (a.deps/name/'CMakeLists.txt').is_file():
            p.error('Missing dependency source: '+str(a.deps/name))
    stage.mkdir()
    copy_tree(PKG,stage/'neupan_cpp_ros1')
    for name in ('osqp','osqp-eigen','qdldl'):
        copy_tree(a.deps/name,stage/'deps'/name)
    shutil.copy2(PKG.parent/'LICENSE',stage/'LICENSE')
    shutil.copy2(a.readme,stage/'README_IPC_CN.md')
    versions=json.loads(a.source_versions.read_text())
    versions.update(bundle=a.name,package='neupan_cpp_ros1',kinematics=['omni','diff'],
                    source_only=True,core='neupan_cpp_ros1/core',
                    validation='neupan_cpp_ros1/TURN_AND_RANDOM_VALIDATION_CN.md',
                    profile='neupan_cpp_ros1/config/fw_mini_omni.yaml',
                    profile_sha256=hashlib.sha256((PKG/'config/fw_mini_omni.yaml').read_bytes()).hexdigest())
    (stage/'SOURCE_VERSIONS.json').write_text(json.dumps(versions,indent=2)+'\n')
    entries=[]
    for f in sorted(stage.rglob('*')):
        if f.is_file():
            entries.append(hashlib.sha256(f.read_bytes()).hexdigest()+'  '+str(f.relative_to(stage)))
    (stage/'MANIFEST.sha256').write_text('\n'.join(entries)+'\n')
    # Normalize ownership and gzip timestamp for a portable source snapshot.
    def normalize(info):
        info.uid=info.gid=0
        info.uname=info.gname=''
        return info
    with archive.open('wb') as raw:
        with gzip.GzipFile(filename='',mode='wb',fileobj=raw,mtime=0) as gz:
            with tarfile.open(fileobj=gz,mode='w') as tf:
                tf.add(stage,arcname=a.name,filter=normalize)
    digest=hashlib.sha256(archive.read_bytes()).hexdigest()
    archive.with_name(archive.name+'.sha256').write_text(digest+'  '+archive.name+'\n')
    print(json.dumps(dict(archive=str(archive),bytes=archive.stat().st_size,
                         sha256=digest,manifest_files=len(entries)),indent=2))


if __name__=='__main__':
    main()

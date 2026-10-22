#!/usr/bin/env python3
"""Build locked Ceres locally, with system Eigen and bundled minimal logging."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs',type=int,default=4)
    args=parser.parse_args()
    if not 1<=args.jobs<=256:
        parser.error('--jobs must be in [1,256]')
    lock=json.loads((ROOT/'dependencies.json').read_text())['ceres']
    archive=ROOT/'.deps'/lock['archive']
    with archive.open('rb') as stream:
        if hashlib.file_digest(stream,'sha256').hexdigest()!=lock['sha256']:
            raise RuntimeError('Ceres archive hash mismatch')
    source=ROOT/'.deps'/('ceres-source-'+lock['commit'])
    if not source.exists():
        with tempfile.TemporaryDirectory(dir=ROOT/'.deps') as tmp:
            with tarfile.open(archive) as tar: tar.extractall(tmp,filter='data')
            (Path(tmp)/('ceres-solver-'+lock['commit'])).rename(source)
    build=ROOT/'.deps'/('ceres-build-'+lock['commit'])
    install=ROOT/'.deps'/('ceres-install-'+lock['commit'])
    options=dict(CMAKE_BUILD_TYPE='Release',CMAKE_CXX_STANDARD='17',CMAKE_INSTALL_PREFIX=str(install),
        CMAKE_CXX_FLAGS='-DEIGEN_MALLOC_ALREADY_ALIGNED=0',
        BUILD_SHARED_LIBS='OFF',BUILD_TESTING='OFF',BUILD_EXAMPLES='OFF',BUILD_BENCHMARKS='OFF',
        MINIGLOG='ON',GFLAGS='OFF',SUITESPARSE='OFF',LAPACK='OFF',USE_CUDA='OFF',
        EIGENSPARSE='ON',SCHUR_SPECIALIZATIONS='OFF',EXPORT_BUILD_DIR='OFF')
    (install/'calib-provenance.json').unlink(missing_ok=True)
    subprocess.run(['cmake','-S',str(source),'-B',str(build),'-G','Ninja',
                    *[f'-D{k}={v}' for k,v in options.items()]],check=True)
    subprocess.run(['cmake','--build',str(build),'--parallel',str(args.jobs)],check=True)
    subprocess.run(['cmake','--install',str(build)],check=True)
    (install/'calib-provenance.json').write_text(json.dumps({'dependency':lock,'cmake_options':options},indent=2)+'\n')


if __name__=='__main__': main()

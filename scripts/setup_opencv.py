#!/usr/bin/env python3
"""Build the locked OpenCV calibration modules locally; no sudo or network here."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--jobs', type=int, default=4)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 256:
        parser.error('--jobs must be in [1,256]')
    lock = json.loads((ROOT / 'dependencies.json').read_text())['opencv']
    archive = ROOT / '.deps' / lock['archive']
    with archive.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != lock['sha256']:
            raise RuntimeError('OpenCV archive hash mismatch; run fetch_dependencies.py')
    source = ROOT / '.deps' / ('opencv-source-' + lock['commit'])
    if not source.exists():
        with tempfile.TemporaryDirectory(dir=ROOT / '.deps') as temporary:
            with tarfile.open(archive) as tar:
                tar.extractall(temporary, filter='data')
            (Path(temporary) / ('opencv-' + lock['commit'])).rename(source)
    build = ROOT / '.deps' / ('opencv-build-' + lock['commit'])
    install = ROOT / '.deps' / ('opencv-install-' + lock['commit'])
    options = dict(CMAKE_BUILD_TYPE='Release', CMAKE_CXX_STANDARD='17',
                   CMAKE_INSTALL_PREFIX=str(install), BUILD_LIST='calib',
                   CPU_BASELINE='SSE2', CPU_DISPATCH='', BUILD_SHARED_LIBS='OFF',
                   BUILD_TESTS='OFF', BUILD_PERF_TESTS='OFF', BUILD_EXAMPLES='OFF',
                   BUILD_opencv_apps='OFF', BUILD_opencv_python3='OFF',
                   BUILD_JAVA='OFF', BUILD_opencv_js='OFF',
                   WITH_IPP='OFF', WITH_ITT='OFF', WITH_OPENCL='OFF', WITH_PROTOBUF='OFF',
                   WITH_OPENMP='OFF', WITH_TBB='OFF', WITH_PTHREADS_PF='OFF',
                   WITH_LAPACK='OFF', WITH_EIGEN='OFF', ENABLE_FAST_MATH='OFF',
                   OPENCV_GENERATE_PKGCONFIG='OFF')
    (install / 'calib-provenance.json').unlink(missing_ok=True)
    subprocess.run(['cmake', '-S', str(source), '-B', str(build), '-G', 'Ninja',
                    *[f'-D{k}={v}' for k, v in options.items()]], check=True)
    subprocess.run(['cmake', '--build', str(build), '--parallel', str(args.jobs)], check=True)
    subprocess.run(['cmake', '--install', str(build)], check=True)
    (install / 'calib-provenance.json').write_text(json.dumps(
        {'dependency': lock, 'cmake_options': options}, indent=2) + '\n')
    print('OpenCV calibration modules ready:', install)


if __name__ == '__main__':
    main()

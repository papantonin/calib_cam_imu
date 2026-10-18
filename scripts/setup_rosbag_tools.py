#!/usr/bin/env python3
"""Install locked extraction tools locally, without sudo or a system pip."""
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import urllib.request

ROOT=Path(__file__).resolve().parents[1]

def main():
    if sys.version_info[:2] != (3,14) or sys.platform != 'linux' or platform.machine() != 'x86_64':
        raise RuntimeError('Current extraction lock targets Python 3.14 on Linux x86_64; regenerate the wheel lock for another platform.')
    cache=ROOT/'.deps'
    cache.mkdir(exist_ok=True)
    lock=json.loads((ROOT/'scripts/pip-bootstrap.json').read_text())
    bootstrap=cache/'pip-bootstrap.whl'
    if not bootstrap.exists() or hashlib.sha256(bootstrap.read_bytes()).hexdigest()!=lock['sha256']:
        with urllib.request.urlopen(lock['url'],timeout=60) as response:
            data=response.read()
        if hashlib.sha256(data).hexdigest()!=lock['sha256']:
            raise RuntimeError('pip bootstrap SHA-256 mismatch')
        with tempfile.NamedTemporaryFile(dir=cache,delete=False) as temporary:
            temporary.write(data)
        Path(temporary.name).replace(bootstrap)
    environment=cache/'rosbag-venv'
    subprocess.run([sys.executable,'-m','venv','--without-pip',str(environment)],check=True)
    env=dict(os.environ,PYTHONPATH=str(bootstrap),PIP_DISABLE_PIP_VERSION_CHECK='1')
    subprocess.run([str(environment/'bin/python'),'-m','pip','install','--require-hashes','--only-binary=:all:',
                    '-r',str(ROOT/'requirements-rosbag.lock')],env=env,check=True)
    print(f'Extraction environment ready: {environment}/bin/python')

if __name__=='__main__': main()

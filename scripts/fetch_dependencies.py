#!/usr/bin/env python3
"""Fetch exact source archives; never update the lock implicitly."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    cache = ROOT / '.deps'
    cache.mkdir(exist_ok=True)
    for name, dep in json.loads((ROOT / 'dependencies.json').read_text()).items():
        target = cache / dep['archive']
        if target.exists() and digest(target) == dep['sha256']:
            print(f'{name} {dep["version"]}: cached, SHA-256 verified')
            continue
        with tempfile.NamedTemporaryFile(dir=cache, suffix='.download') as temp:
            subprocess.run(['curl', '--fail', '--location', '--retry', '3',
                            '--connect-timeout', '15', '--max-time', '300',
                            dep['url'], '--output', temp.name], check=True)
            if digest(Path(temp.name)) != dep['sha256']:
                raise RuntimeError(f'{name}: SHA-256 mismatch; archive not installed')
            # A completed, verified archive is published atomically.
            with Path(temp.name).open('rb') as src:
                with tempfile.NamedTemporaryFile(dir=cache, delete=False) as ready:
                    import shutil
                    shutil.copyfileobj(src, ready)
            Path(ready.name).replace(target)
        print(f'{name} {dep["version"]}: downloaded, SHA-256 verified')


if __name__ == '__main__':
    main()

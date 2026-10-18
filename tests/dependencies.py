#!/usr/bin/env python3
"""Check source lock enforcement without network or administrator access."""
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('fetch_dependencies',Path(__file__).resolve().parents[1]/'scripts'/'fetch_dependencies.py')
fetch=importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetch)

class LockTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        root=Path(self.folder.name)
        self.root_patch=patch.object(fetch,'ROOT',root)
        self.root_patch.start()
        self.addCleanup(self.root_patch.stop)
        (root/'.deps').mkdir()
        self.path=root/'.deps'/'source.tar.gz'
        self.content=b'verified test archive bytes'
        (root/'dependencies.json').write_text(json.dumps({'test':{'version':'1','archive':'source.tar.gz','sha256':hashlib.sha256(self.content).hexdigest(),'url':'https://invalid.example/source'}}))

    def execute(self):
        with contextlib.redirect_stdout(io.StringIO()): fetch.main()

    def test_verified_cache_does_not_download(self):
        self.path.write_bytes(self.content)
        with patch.object(fetch.subprocess,'run') as download:
            self.execute()
            download.assert_not_called()

    def test_corrupt_cache_replaced_only_with_verified_download(self):
        self.path.write_bytes(b'corrupt')
        def download(args,**kwargs): Path(args[-1]).write_bytes(self.content)
        with patch.object(fetch.subprocess,'run',side_effect=download): self.execute()
        self.assertEqual(self.path.read_bytes(),self.content)
        self.assertEqual(list(self.path.parent.iterdir()),[self.path])

    def test_corrupt_download_preserves_existing_file(self):
        self.path.write_bytes(b'previous')
        def download(args,**kwargs): Path(args[-1]).write_bytes(b'wrong digest')
        with patch.object(fetch.subprocess,'run',side_effect=download):
            with self.assertRaisesRegex(RuntimeError,'SHA-256 mismatch'): self.execute()
        self.assertEqual(self.path.read_bytes(),b'previous')
        self.assertEqual(list(self.path.parent.iterdir()),[self.path])

    def test_interrupted_download_not_published(self):
        with patch.object(fetch.subprocess,'run',side_effect=subprocess.CalledProcessError(1,'curl')):
            with self.assertRaises(subprocess.CalledProcessError): self.execute()
        self.assertEqual(list(self.path.parent.iterdir()),[])

if __name__=='__main__': unittest.main()

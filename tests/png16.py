"""Independent raw PNG fixtures: exact high-byte mapping without hidden gamma."""
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
import zlib

def chunk(name,data):
    return struct.pack('>I',len(data))+name+data+struct.pack('>I',zlib.crc32(name+data))

with tempfile.TemporaryDirectory() as directory:
    path=Path(directory)/'samples.png'
    samples=[0,1,255,256,257,4095,4096,32767,32768,65520,65535]
    header=chunk(b'IHDR',struct.pack('>IIBBBBB',len(samples),1,16,0,0,0,0))
    payload=chunk(b'IDAT',zlib.compress(b'\0'+struct.pack('>'+'H'*len(samples),*samples)))
    for gamma in [b'',chunk(b'gAMA',struct.pack('>I',45455)),chunk(b'gAMA',struct.pack('>I',100000))]:
        path.write_bytes(b'\x89PNG\r\n\x1a\n'+header+gamma+payload+chunk(b'IEND',b''))
        result=subprocess.run([sys.argv[1],str(path)],capture_output=True)
        assert result.returncode==0,result.stderr
        assert result.stdout==bytes(value>>8 for value in samples)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+header+chunk(b'IDAT',b'broken deflate')+chunk(b'IEND',b''))
    result=subprocess.run([sys.argv[1],str(path)],capture_output=True)
    assert result.returncode==2,result.stderr
print('mono16 endpoints, byte boundaries, gamma invariance and corruption passed')

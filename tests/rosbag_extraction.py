"""Optional ROS1 synthetic roundtrip tests; run in the locked extraction venv."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from PIL import Image
from rosbags.rosbag1 import Writer
from rosbags.typesys import get_typestore, Stores

spec=importlib.util.spec_from_file_location('extract',Path(__file__).resolve().parents[1]/'scripts/extract_rosbag.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
store=get_typestore(Stores.ROS1_NOETIC)
T=store.types

def header(n):
    return T['std_msgs/msg/Header'](n,T['builtin_interfaces/msg/Time'](1700000000,n),'sensor')

def image(n,big=True):
    # Two pixels and two padding bytes per row; known endian-sensitive samples.
    samples=np.array([[1,256],[4095,65535]],dtype='>u2' if big else '<u2')
    payload=b''.join(row.tobytes()+b'\xaa\xbb' for row in samples)
    return T['sensor_msgs/msg/Image'](header(n),2,2,'mono16',int(big),6,np.frombuffer(payload,dtype='u1'))

def imu(n):
    vector=T['geometry_msgs/msg/Vector3']; quat=T['geometry_msgs/msg/Quaternion']
    return T['sensor_msgs/msg/Imu'](header(n),quat(0.,0.,0.,1.),np.zeros(9),vector(.1,-.2,.3),np.zeros(9),vector(1.,2.,9.81),np.zeros(9))

def bag(path,duplicate=False):
    with Writer(path) as writer:
        camera=writer.add_connection('/cam0/image_raw','sensor_msgs/msg/Image',typestore=store)
        motion=writer.add_connection('/imu0','sensor_msgs/msg/Imu',typestore=store)
        for n in [1,2]:
            msg=image(1 if duplicate else n,n==1)
            writer.write(camera,1700000001000000000+n,store.serialize_ros1(msg,camera.msgtype))
            writer.write(motion,1700000001000000000+n,store.serialize_ros1(imu(n),motion.msgtype))

def camera_only_bag(path, start, topic='/cam0/image_raw'):
    with Writer(path) as writer:
        connection=writer.add_connection(topic,'sensor_msgs/msg/Image',typestore=store)
        for offset in (0,1):
            msg=image(0)
            msg.header.stamp.sec=0
            writer.write(connection,start+offset,store.serialize_ros1(msg,connection.msgtype))

class Tests(unittest.TestCase):
    def test_bayer_bilinear_independent_neighbors(self):
        rng=np.random.default_rng(85)
        def color(y,x): return ((0,1),(1,2))[y%2][x%2]
        for height,width in [(2,2),(3,5),(6,7)]:
            raw=rng.integers(0,256,(height,width),dtype='uint8')
            converted=module.debayer_rggb8(raw)
            for y in range(height):
                for x in range(width):
                    for channel in range(3):
                        if color(y,x)==channel: expected=int(raw[y,x])
                        else:
                            neighbors=[int(raw[j,i]) for j in range(max(0,y-1),min(height,y+2))
                                       for i in range(max(0,x-1),min(width,x+2))
                                       if color(j,i)==channel and (channel!=1 or j==y or i==x)]
                            expected=(sum(neighbors)+len(neighbors)//2)//len(neighbors)
                        self.assertEqual(int(converted[y,x,channel]),expected)
        raw=np.array([[200,100],[100,20]],dtype='u1')
        np.testing.assert_array_equal(module.debayer_rggb8(raw),np.tile([200,100,20],(2,2,1)))

    def test_bayer_explicit_conversion_padding_and_png(self):
        msg=T['sensor_msgs/msg/Image'](header(1),2,2,'bayer_rggb8',0,3,
                                      np.array([200,100,255,100,20,255],dtype='u1'))
        with self.assertRaisesRegex(ValueError,'explicit'): module.image_array(msg)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'cam0').mkdir()
            result=module.write_frame(root,'cam0',1,2,msg,bayer_conversion='bilinear')
            with Image.open(root/'cam0/1.png') as png:
                self.assertEqual(png.mode,'RGB')
                np.testing.assert_array_equal(np.asarray(png),np.tile([200,100,20],(2,2,1)))
            self.assertEqual(result['conversion'],'rggb8_to_rgb8_bilinear_available_neighbors')
            source=root/'bayer.bag'
            with Writer(source) as writer:
                connection=writer.add_connection('/cam0/image_raw','sensor_msgs/msg/Image',typestore=store)
                writer.write(connection,1_700_000_000_000_000_002,store.serialize_ros1(msg,connection.msgtype))
            module.extract(source,root/'native',without_imu=True,bayer_conversion='bilinear')
            audit_spec=importlib.util.spec_from_file_location('audit',Path(__file__).resolve().parents[1]/'scripts/audit_extracted_dataset.py')
            audit_module=importlib.util.module_from_spec(audit_spec);audit_spec.loader.exec_module(audit_module)
            for workers in (1,2):
                self.assertEqual(audit_module.audit(root/'native',workers)['counts'],{'cam0':1})
            (root/'native/cam0/1700000000000000001.png').write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError,'PNG digest'): audit_module.audit(root/'native')
        with self.assertRaises(ValueError): module.debayer_rggb8(np.zeros((1,2),dtype='u1'))

    def test_multi_bag_camera_only_explicit_record_times(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);first=root/'first.bag';second=root/'second.bag'
            camera_only_bag(first,1_700_000_000_000_000_000)
            camera_only_bag(second,1_700_000_000_000_000_100,topic='/cam1/image_raw')
            with self.assertRaisesRegex(ValueError,'one IMU'):
                module.extract([first,second],root/'requires-imu')
            with self.assertRaisesRegex(ValueError,'non-increasing header'):
                module.extract([first,second],root/'requires-stamps',without_imu=True)
            result=module.extract([second,first],root/'native',without_imu=True,timestamp_source='bag',workers=2)
            self.assertFalse((root/'native/imu0').exists())
            self.assertFalse(result['imu_present'])
            self.assertEqual(len(result['sources']),2)
            self.assertEqual(result['streams']['cam0']['zero_header_timestamps'],2)
            self.assertEqual(result['streams']['cam0']['min_bag_minus_header_ns'],1_700_000_000_000_000_000)
            records=[json.loads(line) for line in (root/'native/frames.jsonl').read_text().splitlines()]
            self.assertEqual([r['timestamp_ns'] for r in records],
                             [1_700_000_000_000_000_000,1_700_000_000_000_000_001,
                              1_700_000_000_000_000_100,1_700_000_000_000_000_101])
            self.assertTrue(all(r['header_timestamp_ns']==0 for r in records))
            self.assertEqual(records[0]['source_bag'],str(first))
            for record in records:
                with Image.open(root/'native'/record['path']) as png:
                    np.testing.assert_array_equal(np.asarray(png),[[1,256],[4095,65535]])

    def test_overlap_duplicate_source_and_conflicting_imu_options(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);a=root/'a.bag';b=root/'b.bag'
            camera_only_bag(a,100);camera_only_bag(b,101)
            with self.assertRaisesRegex(ValueError,'Overlapping'):
                module.extract([a,b],root/'overlap',without_imu=True,timestamp_source='bag')
            with self.assertRaisesRegex(ValueError,'distinct'):
                module.extract([a,a],root/'duplicate',without_imu=True,timestamp_source='bag')
            with self.assertRaisesRegex(ValueError,'mutually exclusive'):
                module.extract(a,root/'conflict',without_imu=True,imu_topic='/imu0')

    def test_endian_padding(self):
        for endian in [True,False]:
            np.testing.assert_array_equal(module.image_array(image(1,endian)),[[1,256],[4095,65535]])
    def test_bad_payload_and_timestamp(self):
        msg=image(1);msg.step=3
        with self.assertRaises(ValueError): module.image_array(msg)
        with self.assertRaises(ValueError): module.timestamp_ns(T['builtin_interfaces/msg/Time'](1,1_000_000_000))
    def test_bgr(self):
        msg=T['sensor_msgs/msg/Image'](header(1),1,1,'bgr8',0,3,np.array([1,2,3],dtype='u1'))
        np.testing.assert_array_equal(module.image_array(msg),[[[3,2,1]]])
    def test_roundtrip_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'test.bag';bag(source)
            result=module.extract(source,root/'native',workers=2)
            self.assertEqual(result['streams']['cam0']['messages'],2)
            self.assertEqual(result['streams']['cam0']['min_bag_minus_header_ns'],1_000_000_000)
            with Image.open(root/'native/cam0/1700000000000000001.png') as png:
                np.testing.assert_array_equal(np.asarray(png),[[1,256],[4095,65535]])
            csv=(root/'native/imu0/imu0.csv').read_text().splitlines()
            self.assertEqual(csv[1].split(',')[0],'1700000000000000001')
            self.assertEqual([float(v) for v in csv[1].split(',')[1:]],[.1,-.2,.3,1.,2.,9.81])
            records=[json.loads(line) for line in (root/'native/frames.jsonl').read_text().splitlines()]
            self.assertEqual(records[0]['pixel_sha256'],records[1]['pixel_sha256'])
            with self.assertRaises(FileExistsError): module.extract(source,root/'native')
    def test_duplicate_is_incomplete(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);source=root/'bad.bag';bag(source,duplicate=True)
            with self.assertRaisesRegex(ValueError,'non-increasing'): module.extract(source,root/'native')
            self.assertFalse((root/'native').exists())
            self.assertTrue((root/'native.incomplete/error.json').exists())

if __name__=='__main__': unittest.main()

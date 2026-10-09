import csv, hashlib, json, os, pathlib, sqlite3, subprocess, sys, yaml
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
src, out = map(pathlib.Path, sys.argv[1:])
out.mkdir(parents=True, exist_ok=False)
meta=yaml.safe_load((src/'metadata.yaml').read_text())['rosbag2_bagfile_information']
keep={'/clock','/tf','/tf_static','/amr/sensors/merged_lidar/scan'}
w=rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=str(out/'input_bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('cdr','cdr'))
for item in meta['topics_with_message_count']:
    t=item['topic_metadata']
    if t['name'] in keep:
        w.create_topic(rosbag2_py.TopicMetadata(name=t['name'],type=t['type'],serialization_format='cdr',offered_qos_profiles=t['offered_qos_profiles']))
f=(out/'recorded_events.csv').open('w'); c=csv.writer(f); c.writerow(['kind','recv','sim','stamp','extra'])
receipt=(out/'decode_receipts.jsonl').open('w',buffering=1)
sim=0; counts={}; removed=0
def decoded_chunks():
    files=meta['relative_file_paths']; index=0
    while index<len(files):
        group=[]; compressed=b''
        while index<len(files):
            relative=files[index]; index+=1; group.append(src/relative)
            compressed+=(src/relative).read_bytes()
            result=subprocess.run(['zstd','-dc'],input=compressed,capture_output=True)
            if result.returncode==0: break
        if result.returncode: raise RuntimeError(result.stderr.decode())
        data=result.stdout; offset=0
        for original in group:
            assert data[offset:offset+16]==b'SQLite format 3\0'
            page=int.from_bytes(data[offset+16:offset+18],'big'); page=65536 if page==1 else page
            size=page*int.from_bytes(data[offset+28:offset+32],'big')
            assert size>0 and offset+size<=len(data)
            yield original,data[offset:offset+size],len(group)
            offset+=size
        assert offset==len(data)
for index, (original, decoded, group_size) in enumerate(decoded_chunks()):
    scratch=out/'decode_scratch.db3'
    scratch.write_bytes(decoded)
    db=sqlite3.connect(f'file:{scratch}?mode=ro',uri=True)
    assert db.execute('pragma quick_check').fetchall()==[('ok',)]
    topics={i:n for i,n in db.execute('select id,name from topics') if n in keep or n=='/map'}
    query='select topic_id,timestamp,data from messages where topic_id in (%s) order by timestamp' % ','.join(map(str,topics))
    for tid, ts, data in db.execute(query):
        n=topics[tid]; recv=ts/1e9
        if n=='/clock':
            m=deserialize_message(data,Clock); sim=m.clock.sec+m.clock.nanosec/1e9
        elif n=='/tf':
            m=deserialize_message(data,TFMessage)
            for t in m.transforms:
                if (t.header.frame_id.lstrip('/'),t.child_frame_id.lstrip('/'))==('map','odom'):
                    c.writerow(['mo',recv,sim,t.header.stamp.sec+t.header.stamp.nanosec/1e9,''])
            filtered=[t for t in m.transforms if (t.header.frame_id.lstrip('/'),t.child_frame_id.lstrip('/'))!=('map','odom')]
            removed+=len(m.transforms)-len(filtered)
            if not filtered: continue
            m.transforms=filtered; data=serialize_message(m)
        elif n.endswith('/scan'):
            m=deserialize_message(data,LaserScan); c.writerow(['merged',recv,sim,m.header.stamp.sec+m.header.stamp.nanosec/1e9,len(m.ranges)])
        elif n=='/map':
            m=deserialize_message(data,OccupancyGrid)
            c.writerow(['map',recv,sim,m.header.stamp.sec+m.header.stamp.nanosec/1e9,f'{m.info.width}x{m.info.height} {m.data.count(0)} {m.info.resolution}'])
        if n in keep: w.write(n,data,ts); counts[n]=counts.get(n,0)+1
    db.close()
    receipt.write(json.dumps({'source':str(original),'source_sha256':hashlib.sha256(original.read_bytes()).hexdigest(),'frame_group_files':group_size,'scratch_bytes':scratch.stat().st_size,'removed_only_derived_scratch':str(scratch)})+'\n')
    scratch.unlink()
    if index%20==0: print(index,counts,flush=True)
del w
f.close(); receipt.close()
(out/'extract_result.json').write_text(json.dumps({'counts':counts,'map_odom_removed':removed},indent=2))
print(counts,removed)

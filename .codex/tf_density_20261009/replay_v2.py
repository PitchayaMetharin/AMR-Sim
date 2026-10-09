import argparse, csv, json, os, pathlib, signal, subprocess, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from rosbag2_interfaces.srv import Resume
p=argparse.ArgumentParser();p.add_argument('name');p.add_argument('--overlay');p.add_argument('--load',type=int,default=2);a=p.parse_args()
ws=pathlib.Path(os.environ['AMR_WS']); root=ws/'.ros_logs/tf_density_full_20261009';out=root/a.name;out.mkdir(exist_ok=False)
env=dict(os.environ,ROS_DOMAIN_ID='230',ROS_LOG_DIR=str(out/'roslog'),ROS_HOME=str(out/'roshome'),TMPDIR=str(out/'tmp'));pathlib.Path(env['TMPDIR']).mkdir();pathlib.Path(env['ROS_HOME']).mkdir();os.environ.update(env)
procs=[]; logs=[]; receipt=[]; stopped=False
interrupted=False
def on_term(signum,frame):
 global interrupted
 interrupted=True
signal.signal(signal.SIGTERM,on_term)
def check_owned():
 if interrupted: raise RuntimeError('replay terminated')
 for name,proc in procs:
  if name!='play' and proc.poll() is not None: raise RuntimeError('owned '+name+' exited')

def start(name,argv):
 f=(out/(name+'.log')).open('w');logs.append(f);p=subprocess.Popen(argv,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append((name,p));receipt.append({'name':name,'pid':p.pid,'argv':argv});return p

def call(client,request,seconds=45):
 end=time.monotonic()+seconds
 while time.monotonic()<end and not client.wait_for_service(timeout_sec=.2):
  check_owned()
  if any(proc.poll() is not None for name,proc in procs if name in ('slam','monitor','play')):raise RuntimeError('owned node exited during readiness')
 if not client.service_is_ready():raise RuntimeError('service not ready '+client.srv_name)
 future=client.call_async(request)
 while time.monotonic()<end and not future.done():
  check_owned();rclpy.spin_once(n,timeout_sec=.1)
 if not future.done():raise RuntimeError('service timeout '+client.srv_name)
 return future.result()
try:
 start('monitor',['python3',str(ws/'.ros_logs/tf_replay_20261009/scripts/monitor.py'),str(out)])
 for i in range(a.load):start('cpu'+str(i),['python3','-c','while True: pass'])
 argv=['ros2','run','slam_toolbox','async_slam_toolbox_node','--ros-args','-r','__ns:=/amr','-r','__node:=slam_toolbox','--params-file',str(ws/'src/amr_slam/config/mapper.yaml')]
 if a.overlay:argv+=['--params-file',str(pathlib.Path(a.overlay).resolve())]
 slam=start('slam',argv)
 rclpy.init();n=Node('replay_control')
 client=n.create_client(GetParameters,'/amr/slam_toolbox/get_parameters');req=GetParameters.Request();req.names=['minimum_time_interval','minimum_travel_distance','minimum_travel_heading','map_update_interval','transform_timeout']
 values=call(client,req);(out/'params.json').write_text(json.dumps({'names':req.names,'values':[v.double_value for v in values.values]},indent=2))
 play=start('play',['ros2','bag','play',str(root/'input_bag'),'--start-paused','--disable-keyboard-controls','--qos-profile-overrides-path',str(ws/'.ros_logs/tf_replay_20261009/scripts/qos_play.yaml'),'--read-ahead-queue-size','5000'])
 call(n.create_client(Resume,'/rosbag2_player/resume'),Resume.Request())
 begin=time.monotonic(); next_print=begin
 while play.poll() is None:
  check_owned()
  rclpy.spin_once(n,timeout_sec=.2)
  now=time.monotonic()
  if now>=next_print:print(a.name,'elapsed',round(now-begin),'eventbytes',(out/'events.csv').stat().st_size,flush=True);next_print=now+30
  if now-begin>880:raise RuntimeError('replay timeout')
 if play.returncode:raise RuntimeError('play failed '+str(play.returncode))
 end=time.monotonic()+8
 expected=json.loads((root/'extract_result.json').read_text())['counts']['/amr/sensors/merged_lidar/scan']
 drained=False
 while time.monotonic()<end:
  check_owned();rclpy.spin_once(n,timeout_sec=.1)
  rows=list(csv.DictReader((out/'events.csv').open()))
  scans=[r for r in rows if r['kind']=='merged'];tf=[r for r in rows if r['kind']=='mo'];maps=[r for r in rows if r['kind']=='map']
  if len(scans)==expected and tf and maps:
   last=float(scans[-1]['stamp'])
   if float(tf[-1]['stamp'])>=last+1-1e-6 and float(maps[-1]['stamp'])>=last-1e-6:
    drained=True;break
 if not drained:raise RuntimeError('incomplete receipts/final SLAM stamp/map drain')
 (out/'result.json').write_text(json.dumps({'play_exit':play.returncode,'elapsed':time.monotonic()-begin,'complete_input_and_final_map':drained}))
finally:
 for name,proc in procs:
  if proc.poll() is None:
   try:os.killpg(proc.pid,signal.SIGINT)
   except ProcessLookupError:pass
 deadline=time.monotonic()+12
 while any(proc.poll() is None for _,proc in procs) and time.monotonic()<deadline:time.sleep(.1)
 for name,proc in procs:
  if proc.poll() is None:
   try:os.killpg(proc.pid,signal.SIGTERM)
   except ProcessLookupError:pass
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
  for item in receipt:
   if item['name']==name:item['exit']=proc.returncode
 (out/'commands_shutdown.json').write_text(json.dumps(receipt,indent=2))
 for f in logs:f.close()
 if rclpy.ok():rclpy.shutdown()

"""Genesis-side ROS2 process launcher. Agents have no simulator references.

Pipes connect only Genesis to its ROS adapter; all robot input/output crosses
DDS between the bridge and independent agent processes. No direct fallback.
"""
import json,os,queue,subprocess,threading,time,uuid
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
from xrobocon.ros2_wire import decode_command

class RemoteRobotAgent:
    def __init__(self,template):self.motion=template.motion;self.reset()
    def reset(self):
        self.strategy_label=None;self.remaining_coin_ids=[];self.replan_reason=None;self.decision_pending=False;self.decision_error=None;self.target_id=None;self.phase='waiting_for_ros2';self.stage=0;self.done=False;self.checkpoints=[];self.plan_event=None;self.avoidance=SimpleNamespace(event=None)
    def act(self,*args):raise RuntimeError('ROS2 agents must receive observations via ROS2 topics')
    def update(self,metadata):
        for key in ('phase','stage','done','checkpoints','plan_event'):setattr(self,key,metadata[key])
        for key in ('decision_pending','decision_error','target_id','strategy_label','remaining_coin_ids','replan_reason'):setattr(self,key,metadata.get(key))
        self.avoidance.event=metadata['avoidance_event']

class Ros2Transport:
    def __init__(self,config_path,output,options=None):
        options=dict(options or {});self.timeout=float(options.get('timeout',5.));domain=options.get('domain_id',37)
        if not 0<self.timeout<120 or type(domain) is not int or not 0<=domain<=232:raise ValueError('invalid ROS2 timeout/domain')
        root=Path(__file__).resolve().parents[1];self.process=None;self.log=None
        from xrobocon.storage import resource_root
        resources=resource_root()
        self.responses=queue.Queue();self.counter=0;self.epoch=uuid.uuid4().hex;self.last_exchange={}
        self.log=(Path(output)/'ros2-processes.log').open('w')
        env=dict(os.environ,ROS_DOMAIN_ID=str(domain),RMW_IMPLEMENTATION='rmw_cyclonedds_cpp',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',RCUTILS_LOGGING_USE_STDOUT='0',PYTHONUNBUFFERED='1')
        env['PIXI_CACHE_DIR']=str(resources/'pixi-cache')
        env.pop('ROS_LOCALHOST_ONLY',None)
        from xrobocon.ros2_runtime import worker_command
        command=worker_command(root,config_path)
        try:
            self.process=subprocess.Popen(command,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,text=True,bufsize=1,env=env)
            def read():
                try:
                    for line in self.process.stdout:
                        try:self.responses.put(json.loads(line))
                        except json.JSONDecodeError:self.responses.put(dict(ok=False,error='invalid ROS2 sidecar protocol: '+line[:160]))
                finally:self.responses.put(dict(ok=False,error='ROS2 process exited'))
            threading.Thread(target=read,daemon=True).start()
            self.runtime=self._receive(60)
            if not self.runtime.get('ready'):raise RuntimeError(str(self.runtime))
            (Path(output)/'ros2-runtime.json').write_text(json.dumps(self.runtime|dict(domain_id=domain,transport='DDS',rmw=env['RMW_IMPLEMENTATION']),indent=2))
        except BaseException:self.close();raise
    def _receive(self,timeout):
        try:response=self.responses.get(timeout=timeout)
        except queue.Empty:raise TimeoutError('ROS2 command not received; physics stopped, no direct-data fallback')
        if response.get('ok') is False:raise RuntimeError(response.get('error','ROS2 exchange failed'))
        return response
    def _request(self,data):
        if self.process.poll() is not None:raise RuntimeError('ROS2 bridge is not running')
        self.process.stdin.write(json.dumps(data,allow_nan=False)+'\n');self.process.stdin.flush()
        return self._receive(self.timeout+2)
    def reset(self,participants):
        self._request(dict(op='reset',participants=list(participants),token=uuid.uuid4().hex,timeout=self.timeout))
    def exchange(self,observations,dt):
        self.counter+=1;request_id=f'{self.epoch}:{self.counter}'
        result=self._request(dict(op='exchange',request_id=request_id,timeout=self.timeout,dt=dt,timestamp=next(iter(observations.values())).timestamp,observations={name:asdict(obs) for name,obs in observations.items()}))
        replies=result['responses']
        if set(replies)!=set(observations) or any(r['request_id']!=request_id for r in replies.values()):raise ValueError('mismatched ROS2 replies')
        self.last_exchange=dict(request_id=request_id,agent_pids={n:r['pid'] for n,r in replies.items()},sources={n:r['source'] for n,r in replies.items()},published_sensor_counts=result['published_sensor_counts'],observed_sensor_stamps={n:r['observed_sensor_stamps'] for n,r in replies.items()})
        return {n:decode_command(r['command']) for n,r in replies.items()},{n:r['metadata'] for n,r in replies.items()}
    def close(self):
        process=self.process
        if process is not None and process.poll() is None:
            try:
                process.stdin.write('{"op":"shutdown"}\n');process.stdin.flush();process.wait(timeout=8)
            except (OSError,subprocess.TimeoutExpired):
                process.terminate()
                try:process.wait(timeout=8)
                except subprocess.TimeoutExpired:process.kill();process.wait()
        if self.log is not None:self.log.close();self.log=None

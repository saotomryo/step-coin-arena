"""ROS2-only sidecar and independent robot-agent nodes (no Genesis imports)."""
import argparse,json,os,queue,signal,subprocess,sys,threading,time
from dataclasses import asdict
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from xrobocon.ros2_sensor_messages import TYPES,TOPICS,to_message,from_message,stamp_key,set_stamp
from xrobocon.ros2_wire import encode_observation,decode_observation,decode_command
from xrobocon.sensors import SensorSample,validate_sensor_config
from xrobocon.agent_factory import make_agent

QOS=QoSProfile(depth=50,reliability=ReliabilityPolicy.RELIABLE)

def string_message(value):return String(data=json.dumps(value,allow_nan=False))
def emit(value):print(json.dumps(value,allow_nan=False),flush=True)

class AgentNode(Node):
    def __init__(self,participant,config):
        super().__init__('agent_'+participant,namespace='/xrobocon/'+participant,parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.participant=participant;self.agent=make_agent(config['agents'][participant]);self.samples={};self.pending=None;self.completed=None
        self.commands=self.create_publisher(String,'command',QOS)
        self.twist=self.create_publisher(TwistStamped,'cmd_vel',QOS)
        self.reset_ack=self.create_publisher(String,'reset_ack',QOS)
        self.state_sub=self.create_subscription(String,'observation',self.on_observation,QOS)
        self.reset_sub=self.create_subscription(String,'reset',self.on_reset,QOS)
        self.sensor_subs=[]
        specs=validate_sensor_config(config.get('sensors',{}),config['agents'])
        for kind in specs.get(participant,{}):
            self.sensor_subs.append(self.create_subscription(TYPES[kind],TOPICS[kind],lambda msg,k=kind:self.on_sensor(k,msg),QOS))
    def on_reset(self,msg):
        self.agent.reset();self.samples={};self.pending=None;self.completed=None
        self.reset_ack.publish(msg)
    def on_sensor(self,kind,msg):
        stamp,data=from_message(kind,msg)
        cache=self.samples.setdefault(kind,{})
        cache[stamp]=data
        # A state references the latest exposed sample, at most one tick behind.
        while len(cache)>20:cache.pop(next(iter(cache)))
        self.try_act()
    def on_observation(self,msg):
        self.pending=json.loads(msg.data);self.try_act()
    def try_act(self):
        request=self.pending
        if request is None or request['request_id']==self.completed:return
        data=request['observation'];values={}
        for kind,meta in data['sensors'].items():
            stamp=stamp_key(meta['timestamp'])
            if stamp not in self.samples.get(kind,{}):return
            values[kind]=self.samples[kind][stamp]
        observation=decode_observation(data,values)
        command=self.agent.act(observation,request['dt'])
        # Validate before publishing a malformed controller output to the plant.
        command=decode_command(asdict(command))
        metadata=dict(strategy_label=getattr(self.agent,'strategy_label',None),remaining_coin_ids=getattr(self.agent,'remaining_coin_ids',[]),replan_reason=getattr(self.agent,'replan_reason',None),decision_pending=getattr(self.agent,'decision_pending',False),decision_error=getattr(self.agent,'decision_error',None),target_id=getattr(self.agent,'target_id',None),phase=getattr(self.agent,'phase','coins'),stage=getattr(self.agent,'stage',None),done=getattr(self.agent,'done',False),checkpoints=getattr(self.agent,'checkpoints',[]),plan_event=getattr(self.agent,'plan_event',None),avoidance_event=getattr(getattr(self.agent,'avoidance',None),'event',None))
        response=dict(request_id=request['request_id'],participant=self.participant,command=asdict(command),metadata=metadata,pid=os.getpid(),observed_sensor_stamps={k:s.timestamp for k,s in observation.sensors.items()},source=observation.source)
        twist=TwistStamped();set_stamp(twist.header.stamp,observation.timestamp);twist.header.frame_id=self.participant+'/base_link';twist.twist.linear.x=float(command.linear);twist.twist.angular.z=float(command.angular)
        self.twist.publish(twist);self.commands.publish(string_message(response));self.completed=request['request_id']

class BridgeNode(Node):
    def __init__(self,config):
        super().__init__('genesis_bridge',namespace='/xrobocon')
        self.names=list(config['agents']);self.responses={};self.reset_responses={};self.states={};self.resets={};self.sensors={};self.sent={};self.counts={name:{} for name in self.names}
        self.clock=self.create_publisher(Clock,'/clock',QOS);self.subscriptions_list=[]
        specs=validate_sensor_config(config.get('sensors',{}),self.names)
        for name in self.names:
            prefix='/xrobocon/'+name+'/'
            self.states[name]=self.create_publisher(String,prefix+'observation',QOS)
            self.resets[name]=self.create_publisher(String,prefix+'reset',QOS)
            self.subscriptions_list.append(self.create_subscription(String,prefix+'command',self.on_command,QOS))
            self.subscriptions_list.append(self.create_subscription(String,prefix+'reset_ack',lambda msg,n=name:self.on_ack(n,msg),QOS))
            for kind in specs.get(name,{}):self.sensors[name,kind]=self.create_publisher(TYPES[kind],prefix+TOPICS[kind],QOS)
    def on_command(self,msg):
        response=json.loads(msg.data);name=response['participant']
        if name in self.names:self.responses[name]=response
    def on_ack(self,name,msg):self.reset_responses[name]=json.loads(msg.data)['token']
    def wait(self,predicate,timeout):
        deadline=time.monotonic()+timeout
        while rclpy.ok() and not predicate():
            if time.monotonic()>=deadline:raise TimeoutError('ROS2 topic exchange timed out')
            rclpy.spin_once(self,timeout_sec=.01)
    def ready(self):
        return all(pub.get_subscription_count()>0 for pub in list(self.states.values())+list(self.resets.values())+list(self.sensors.values()))
    def exchange(self,request):
        self.responses={};clock=Clock();set_stamp(clock.clock,request['timestamp']);self.clock.publish(clock)
        for name,data in request['observations'].items():
            for kind,raw in data['sensors'].items():
                key=(name,kind);signature=(raw['sequence'],raw['timestamp'])
                if self.sent.get(key)!=signature:
                    self.sensors[key].publish(to_message(kind,SensorSample(**raw)))
                    self.sent[key]=signature;self.counts[name][kind]=self.counts[name].get(kind,0)+1
            # Strip measurements from JSON; agents must consume standard topics.
            from xrobocon.agent import Observation,PublicCompetition
            from xrobocon.navigation import Pose2D
            obs=Observation(Pose2D(**data['pose']),data['timestamp'],data['source'],PublicCompetition(**data['competition']),data['telemetry'],data['other_robots'],{k:SensorSample(**v) for k,v in data['sensors'].items()})
            self.states[name].publish(string_message(dict(request_id=request['request_id'],dt=request['dt'],observation=encode_observation(obs))))
        self.wait(lambda:all(self.responses.get(n,{}).get('request_id')==request['request_id'] for n in self.names),request['timeout'])
        return dict(responses=self.responses,published_sensor_counts=self.counts)
    def reset_agents(self,request):
        token=request['token'];names=request['participants']
        for name in names:self.resets[name].publish(string_message(dict(token=token)))
        self.wait(lambda:all(self.reset_responses.get(n)==token for n in names),request['timeout'])
        self.sent={key:v for key,v in self.sent.items() if key[0] not in names}
        return dict(reset=names)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--role',choices=('bridge','agent'),required=True);parser.add_argument('--config',required=True);parser.add_argument('--participant');args=parser.parse_args()
    config=json.loads(Path(args.config).read_text());rclpy.init(args=[])
    if args.role=='agent':
        node=AgentNode(args.participant,config)
        signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        try:rclpy.spin(node)
        except (ExternalShutdownException,KeyboardInterrupt):pass
        except RCLError:
            if rclpy.ok():raise
        finally:
            if hasattr(node.agent,'close'):node.agent.close()
            node.destroy_node();rclpy.try_shutdown()
        return
    children=[];node=None
    def interrupted(signum,frame):raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,interrupted)
    try:
        for name in config['agents']:
            children.append(subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--role','agent','--config',str(Path(args.config).resolve()),'--participant',name],stdout=sys.stderr,stderr=sys.stderr))
        node=BridgeNode(config);node.wait(node.ready,30)
        emit(dict(ready=True,bridge_pid=os.getpid(),agent_pids=[p.pid for p in children]))
        requests=queue.Queue()
        def read_stdin():
            for line in sys.stdin:requests.put(json.loads(line))
            requests.put(dict(op='shutdown'))
        threading.Thread(target=read_stdin,daemon=True).start()
        while True:
            try:request=requests.get(timeout=.01)
            except queue.Empty:rclpy.spin_once(node,timeout_sec=.01);continue
            if request['op']=='shutdown':break
            try:
                result=node.exchange(request) if request['op']=='exchange' else node.reset_agents(request)
                emit(dict(ok=True,**result))
            except Exception as error:
                emit(dict(ok=False,error=f'{type(error).__name__}: {error}'))
    finally:
        for child in children:child.terminate()
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:child.kill();child.wait()
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()
if __name__=='__main__':main()

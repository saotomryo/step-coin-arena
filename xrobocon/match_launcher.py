"""Native launcher lifecycle; child owns Genesis and its ROS2 worker processes."""
from pathlib import Path
import json,os,secrets,signal,subprocess,sys,time
from xrobocon.match_setup import build_match_config

class MatchLauncher:
    def __init__(self,catalog,output_root):
        self.library=None;self.catalog=catalog;self.output_root=Path(output_root);self.process=None;self.log=None;self.trial=None;self.result=None;self.preview_process=None;self.preview_log=None
    @property
    def running(self):return self.process is not None and self.process.poll() is None
    def stop_preview(self):
        if self.preview_process is not None and self.preview_process.poll() is None:
            self.preview_process.terminate()
            try:self.preview_process.wait(timeout=5)
            except subprocess.TimeoutExpired:self.preview_process.kill();self.preview_process.wait()
        if self.preview_log is not None:self.preview_log.close();self.preview_log=None
        self.preview_process=None
    def preview(self,robot_id):
        if self.running:raise ValueError('競技中はプレビューを開けません')
        robot=self.catalog.options('robots').get(robot_id)
        if robot is None:raise ValueError('unknown robot')
        asset=(self.catalog.path.parent/robot.get('visual_asset',robot['robot_asset'])).resolve()
        if not asset.is_file():raise ValueError('robot asset missing')
        self.stop_preview()
        # Rendering is a separate process so MuJoCo/GLFW and the Qt UI
        # have independent OpenGL contexts and only one robot is displayed.
        project=Path(__file__).resolve().parents[1]
        if self.output_root.is_absolute() and len(self.output_root.parts)>2 and self.output_root.parts[1]=='Volumes' and not os.path.ismount(Path('/Volumes')/self.output_root.parts[2]):raise RuntimeError('外部SSDを接続してください')
        self.output_root.mkdir(parents=True,exist_ok=True)
        self.preview_log=(self.output_root/'preview.log').open('a')
        command=[sys.executable,str(project/'scripts/preview_robot.py'),'--asset',str(asset),'--label',robot['label']]
        try:self.preview_process=subprocess.Popen(command,cwd=project,stdout=self.preview_log,stderr=subprocess.STDOUT)
        except BaseException:self.preview_log.close();self.preview_log=None;raise
        return self.preview_process
    def launch(self,choices,*,viewer=True,seconds=None,panel='python'):
        if self.running:raise ValueError('a match is already running')
        if self.output_root.is_absolute() and len(self.output_root.parts)>2 and self.output_root.parts[1]=='Volumes':
            volume=Path('/Volumes')/self.output_root.parts[2]
            if not os.path.ismount(volume):raise RuntimeError('外部SSDを接続してください')
        if panel not in ('python','overlay'):raise ValueError('unknown operator panel')
        if panel=='overlay' and not viewer:raise ValueError('overlay requires a viewer')
        config=build_match_config(self.catalog,choices)
        config['desktop_navigation']=dict(catalog=str(self.catalog.path),out_root=str(self.output_root.resolve()))
        if self.library:config['desktop_navigation']['library']=str(Path(self.library).resolve())
        if seconds is not None:config['competition']['time_limit']=seconds
        # Separate simultaneous local matches, including existing viewer sessions.
        config['transport']['domain_id']=100+secrets.randbelow(100)
        self.stop_preview()
        self.trial=self.output_root/f'match-{time.time_ns()}';self.trial.mkdir(parents=True,exist_ok=False)
        config_file=self.trial/'selected-config.json';config_file.write_text(json.dumps(config,ensure_ascii=False,indent=2))
        project=Path(__file__).resolve().parents[1]
        command=[sys.executable,str(project/'scripts/run_competition.py'),'--config',str(config_file),'--out',str(self.trial/'run'),'--autostart','--return-after-match']
        command+=['--panel',panel]
        if not viewer:command.append('--no-viewer')
        self.log=(self.trial/'process.log').open('w');self.result=None
        try:self.process=subprocess.Popen(command,cwd=project,stdout=self.log,stderr=subprocess.STDOUT)
        except BaseException:self.log.close();self.log=None;raise
        return self.trial
    def stop(self):
        if self.running:self.process.send_signal(signal.SIGINT)
    def poll(self):
        if self.process is None or self.running or self.result is not None:return self.result
        if self.log:self.log.close();self.log=None
        state_file=self.trial/'run/session-state.json';error_file=self.trial/'run/error.json'
        state=json.loads(state_file.read_text()) if state_file.exists() else {}
        error=json.loads(error_file.read_text()).get('message') if error_file.exists() else None
        if self.process.returncode and not error:error=f'起動または実行に失敗しました（終了コード {self.process.returncode}）。process.logを確認してください。'
        self.result=dict(scores=state.get('scores',{}),collection_counts=state.get('collection_counts',{}),status=state.get('control_status','error'),error=error,output=str(self.trial))
        (self.trial/'result.json').write_text(json.dumps(self.result,ensure_ascii=False,indent=2));return self.result

def external_match_root():
    from xrobocon.storage import resource_root
    return resource_root()/'evaluation_results'/'matches'

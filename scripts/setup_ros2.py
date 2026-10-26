"""Install the project-local ROS2 runtime with large resources on external SSD.

Run on the host with network access. Never creates an unmounted volume path.
"""
import argparse,hashlib,json,os,platform,subprocess,tarfile,urllib.request,sys
sys.path.insert(0,str(__import__('pathlib').Path(__file__).resolve().parents[1]))
from pathlib import Path

VERSION='0.82.0'
SHA256='3b48bd27de4db31381dc1d5b3d069c98852c0e923aefa30dae0452a088c0810e'

def main():
    p=argparse.ArgumentParser();p.add_argument('--volume');p.add_argument('--resource-root');args=p.parse_args()
    project=Path(__file__).resolve().parents[1]
    volume=Path(args.volume).expanduser().resolve() if args.volume else None
    if volume and (not os.path.ismount(volume) or not os.access(volume,os.W_OK)):
        raise RuntimeError('external volume must be mounted and writable; no internal fallback')
    if args.resource_root:
        from xrobocon.storage import resource_root
        os.environ['XROBOCON_RESOURCE_ROOT']=args.resource_root
        resources=resource_root(project)
    elif volume:resources=volume/'GenesisRobotics-resources'
    else:
        from xrobocon.storage import resource_root
        resources=resource_root(project)
    if volume and not resources.is_relative_to(volume):raise ValueError('resource root must belong to the mounted external volume')
    if platform.system()!='Darwin' or platform.machine()!='arm64':raise RuntimeError('this installer currently targets Apple Silicon macOS')
    resources.mkdir(parents=True,exist_ok=True);(resources/'pixi-cache').mkdir(exist_ok=True);(resources/'ros2-environments').mkdir(exist_ok=True)
    tools=project/'.tools';tools.mkdir(exist_ok=True);pixi=tools/'pixi';downloads=0
    if not pixi.exists():
        archive=resources/f'pixi-{VERSION}.tar.gz'
        urllib.request.urlretrieve(f'https://github.com/prefix-dev/pixi/releases/download/v{VERSION}/pixi-aarch64-apple-darwin.tar.gz',archive);downloads=1
        if hashlib.sha256(archive.read_bytes()).hexdigest()!=SHA256:raise RuntimeError('Pixi checksum mismatch')
        with tarfile.open(archive) as tar:
            with tar.extractfile('pixi') as source,pixi.open('wb') as destination:destination.write(source.read())
        pixi.chmod(0o755)
    installed_version=subprocess.check_output([str(pixi),'--version'],text=True).strip()
    if installed_version!=f'pixi {VERSION}':raise RuntimeError(f'expected Pixi {VERSION}, found {installed_version}')
    local=project/'ros2/.pixi';local.mkdir(exist_ok=True)
    desired=f'detached-environments = "{resources}/ros2-environments"\n[cache]\nroot = "{resources}/pixi-cache"\n'
    old_env=local/'envs';config_file=local/'config.toml'
    if old_env.exists() and not old_env.is_symlink() and (not config_file.exists() or config_file.read_text()!=desired):raise RuntimeError('existing internal ROS2 environment must first be relocated while stopped')
    config_file.write_text(desired)
    (project/'ros2/storage.local.json').write_text(json.dumps(dict(volume=str(volume) if volume else None,resource_root=str(resources)),indent=2))
    command=[str(pixi),'install','--locked','--manifest-path',str(project/'ros2/pixi.toml')]
    runtime_env=dict(os.environ,PIXI_CACHE_DIR=str(resources/'pixi-cache'))
    subprocess.run(command,check=True,env=runtime_env)
    result=subprocess.check_output([str(pixi),'run','--locked','--manifest-path',str(project/'ros2/pixi.toml'),'python','-c','import sys,rclpy; print(sys.prefix)'],text=True,env=runtime_env).strip()
    if not Path(result).resolve().is_relative_to(resources.resolve()):raise RuntimeError('ROS2 prefix is not on configured external storage')
    print(json.dumps(dict(success=True,pixi_version=VERSION,installer_downloads=downloads,ros2_prefix=result,resource_root=str(resources)),indent=2))
if __name__=='__main__':main()

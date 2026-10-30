#!/usr/bin/env python3
"""Theo dõi một job resume SAC; đánh giá ba controller bằng run_ablation_suite.py riêng."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import zipfile


def model_pair(run):
    run = Path(run).resolve()
    for model, replay in (('model.zip', 'replay_buffer.pkl'),
                          ('infrastructure_fault_model.zip', 'infrastructure_fault_replay.pkl'),
                          ('exception_model.zip', 'exception_replay.pkl'),
                          ('interrupted_model.zip', 'interrupted_replay.pkl')):
        if (run/model).is_file() and (run/replay).is_file():
            return run/model, run/replay
    for model in sorted((run/'checkpoints').glob('sac_*_steps.zip'),
                        key=lambda p: int(p.stem.split('_')[1]), reverse=True):
        replay = model.with_name(model.name.replace('sac_', 'sac_replay_buffer_').replace('.zip', '.pkl'))
        if replay.is_file():
            return model, replay
    raise ValueError(f'No paired model/replay in {run}')


def main():
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--resume-train-run',required=True)
    parser.add_argument('--map',required=True)
    parser.add_argument('--output',required=True)
    parser.add_argument('--steps',type=int,default=50000000)
    parser.add_argument('--checkpoint-every',type=int,default=100000)
    parser.add_argument('--device',choices=['cpu','cuda','auto'],default='cuda')
    args=parser.parse_args()
    if min(args.steps,args.checkpoint_every)<=0:parser.error('steps/checkpoint-every must be positive')
    source=Path(args.resume_train_run).resolve();model,replay=model_pair(source)
    with zipfile.ZipFile(model) as archive:completed=json.loads(archive.read('data'))['num_timesteps']
    if args.steps<completed:parser.error('Target below restored timestep count')
    output=Path(args.output).resolve();output.mkdir(parents=True,exist_ok=False)
    status=dict(pid=os.getpid(),state='TRAIN',target_steps=args.steps,phases={},train_log='train.log',
                restored_steps=completed,source_run=str(source))
    def save():
        status['updated_utc']=datetime.now(timezone.utc).isoformat()
        temp=output/'status.tmp';temp.write_text(json.dumps(status,indent=2));temp.replace(output/'status.json')
    save();child=None
    try:
        command=[sys.executable,'-u',str(Path(__file__).parent/'train.py'),'--mode','train',
                 '--map',args.map,'--config',str(source/'config.yaml'),'--resume',str(model),
                 '--replay-buffer',str(replay),'--steps',str(args.steps),
                 '--checkpoint-every',str(args.checkpoint_every),'--device',args.device]
        with (output/'train.log').open('w') as log:
            child=subprocess.Popen(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
            status['child_pid']=child.pid;save()
            for line in child.stdout:
                log.write(line);log.flush()
                if line.startswith('Run: '):status['phases']['train']=line.split(': ',1)[1].strip();save()
            code=child.wait();child.stdout.close()
            if code:raise RuntimeError(f'Train exited {code}; see {output/"train.log"}')
        status['state']='COMPLETE'
    except KeyboardInterrupt:
        if child and child.poll() is None:child.send_signal(signal.SIGINT);child.wait(timeout=120)
        status['state']='INTERRUPTED'
    except Exception as exc:
        status.update(state='FAILED',reason=str(exc));raise
    finally:save()


if __name__=='__main__':main()

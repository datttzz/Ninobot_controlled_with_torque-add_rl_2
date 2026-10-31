#!/usr/bin/env python3
"""Audit log theo AMCL + IMU + encoder; không mở ROS hoặc sửa dữ liệu gốc."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from benchmark_pi_pid_sac import episode_metrics,read_rows,write_csv


def audit_episode(session,reference):
    path=Path(session)/'telemetry.jsonl'
    rows=read_rows(path);analysis=episode_metrics(rows,reference);m=analysis['metrics']
    result=dict(controller=Path(session).parent.name,episode=Path(session).name,outcome=analysis['outcome'],
                terminal_time_s=rows[-1]['simulation_time'],amcl_goal_xy_m=m['amcl_goal_xy_m'],
                amcl_goal_yaw_rad=m['amcl_goal_yaw_rad'],amcl_max_m=m['amcl_max_m'],
                **analysis['task_status'],failure_reasons='; '.join(analysis['mission_failure_reasons']))
    error=np.asarray(analysis['trace']['error']);finite=np.isfinite(error)
    maximum=None
    if finite.any():
        i=int(np.nanargmax(abs(error)))
        maximum=dict(index=i,episode_time_s=rows[i]['simulation_time'],source_stamp_s=rows[i].get('amcl_stamp_s'),
                     signed_error_m=float(error[i]))
    details=dict(source=str(path),raw_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),samples=len(rows),
                 measurement_basis=analysis['measurement_basis'],max_event=maximum,
                 terminal_measurement=analysis['terminal_measurement'],source_measurements=analysis['source_measurements'],
                 missing_reasons=analysis['missing_reasons'],coverage=analysis['coverage'],
                 source_note='Pose: TF AMCL + odometry estimate; body speed: encoder odometry estimate; torque: controller command')
    return result,details


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True,help='Thư mục runner chứa reference_path.json và một hoặc nhiều nhóm')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args(argv)
    reference=json.loads((args.run_root/'reference_path.json').read_text())
    results=[];details=[]
    for path in sorted(args.run_root.glob('*/episode_*/telemetry.jsonl')):
        row,detail=audit_episode(path.parent,reference);results.append(row);details.append(detail)
    if not results:parser.error('Không có telemetry episode; kiểm tra log hạ tầng')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    summary=dict(measurement_basis='AMCL_IMU_ENCODER_v1',thresholds=dict(xy_m=.25,yaw_rad=.25,max_deviation_m=.20),
                 nav2_succeeded=sum(r['nav2_succeeded'] for r in results),
                 amcl_pass=sum(r['amcl_task_pass'] is True for r in results),
                 mission_pass=sum(r['mission_pass'] is True for r in results),
                 mission_unknown=sum(r['mission_pass'] is None for r in results),episodes=results,details=details)
    write_csv(args.output_dir/'episodes.csv',results)
    (args.output_dir/'audit.json').write_text(json.dumps(summary,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('episodes','details')}))


if __name__=='__main__':main()

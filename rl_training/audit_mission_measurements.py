#!/usr/bin/env python3
"""Chẩn đoán sáu log đã có; không chạy hoặc đánh giá thêm checkpoint SAC."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from benchmark_pi_pid_sac import episode_metrics, read_rows, projection
from rl_training.mission_measurement import mission_conditions


def audit_episode(session, reference, registration):
    rows = read_rows(session/'telemetry.jsonl')
    analysis = episode_metrics(rows, reference, map_to_world=registration)
    m = analysis['metrics']; last = rows[-1]
    result = dict(controller=session.parent.name, episode=session.name,
                  outcome=analysis['outcome'], terminal_time_s=last['simulation_time'],
                  terminal_amcl_stamp_s=last.get('amcl_stamp_s'),
                  terminal_gt_stamp_s=last.get('gt_stamp_s'),
                  amcl_goal_xy_m=m['amcl_goal_xy_m'], amcl_goal_yaw_rad=m['amcl_goal_yaw_rad'],
                  gt_goal_xy_m=m['gt_goal_xy_m'], gt_goal_yaw_rad=m['gt_goal_yaw_rad'],
                  amcl_max_m=m['amcl_max_m'], gt_max_m=m['gt_max_m'])
    result.update(mission_conditions(analysis['outcome'], m['amcl_goal_xy_m'],
                  m['amcl_goal_yaw_rad'], m['amcl_max_m'], m['gt_goal_xy_m'],
                  m['gt_goal_yaw_rad'], m['gt_max_m']))
    result['failure_reasons'] = '; '.join(analysis['mission_failure_reasons'])
    details = dict(source=str(session), raw_sha256=hashlib.sha256((session/'telemetry.jsonl').read_bytes()).hexdigest(),
                   first_sample_time_s=rows[0]['simulation_time'], samples=len(rows),
                   requested_goal=last.get('requested_goal'), reference_goal=last.get('reference_goal'),
                   max_events={}, timestamp_checks={}, endpoint_at_each_sources_own_stamp=True,
                   cross_track_checks={}, terminal_stability={})
    points = np.array([[p['x'], p['y']] for p in reference['poses']])
    for label, prefix in [('amcl', 'amcl'), ('gt', 'ground_truth')]:
        error = np.asarray(analysis['trace']['amcl_error' if label=='amcl' else 'error'])
        i = int(np.nanargmax(np.abs(error)))
        stamp_key = label+'_stamp_s'
        result[label+'_max_time_s'] = rows[i]['simulation_time']
        result[label+'_max_source_stamp_s'] = rows[i].get(stamp_key)
        details['max_events'][label] = dict(index=i, episode_time_s=rows[i]['simulation_time'],
                source_stamp_s=rows[i].get(stamp_key), signed_error_m=float(error[i]),
                x=rows[i].get(prefix+'_x'), y=rows[i].get(prefix+'_y'))
        # Trên đường thẳng hiện tại trong đoạn [0,11], e_y phải chính là Y.
        x=np.array([r[prefix+'_x'] for r in rows]);y=np.array([r[prefix+'_y'] for r in rows])
        expected=np.abs(y)  # Đường hiện tại nằm trên trục X; không áp dụng cho đường cong.
        if np.all(points[:,1]==0) and np.all((x>=points[:,0].min()-1e-9)&(x<=points[:,0].max()+1e-9)):
            details['cross_track_checks'][label] = float(np.max(np.abs(np.abs(error)-expected)))
        for axis in ('x','y','yaw'):
            a=last[prefix+'_'+axis];b=rows[-2][prefix+'_'+axis]
            details['terminal_stability'][label+'_'+axis+'_delta_last_50hz']=a-b
    for key in ('simulation_time','amcl_stamp_s','gt_stamp_s','control_stamp_s','imu_stamp_s'):
        values=np.array([r[key] for r in rows if key in r],dtype=float)
        d=np.diff(values)
        details['timestamp_checks'][key]=dict(samples=len(values),nonfinite=int((~np.isfinite(values)).sum()),
                  backwards=int((d<0).sum()), duplicates=int((d==0).sum()), max_gap_s=float(d.max()))
    skew=np.array([r['amcl_stamp_s']-r['gt_stamp_s'] for r in rows])
    details['amcl_minus_gt_stamp_s']=[float(skew.min()),float(skew.max())]
    # Không suy ra quaternion hoặc phase cũ nếu logger chưa ghi.
    details['quaternion_evidence_available']=all('ground_truth_quaternion' in r for r in rows)
    details['episode_state_available']=all('episode_state' in r for r in rows)
    details['source_note']='AMCL = latest TF map->base_footprint, fused localization + encoder odometry; GT = Gazebo model world pose'
    return result,details


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,default=ROOT/'rl_training/runs/fixed3_integration_20261030')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args();args.output_dir.mkdir(parents=True,exist_ok=False)
    reference=json.loads((args.run_root/'reference_path.json').read_text())
    output=[];details=[]
    for group in ('PI','PID','PI_SAC'):
        metadata=json.loads((args.run_root/group/'metadata.json').read_text())
        registration=metadata.get('map_to_world')
        if registration != [0,0,0]:raise ValueError('Audit đường thẳng này yêu cầu đăng ký identity được khai báo')
        for session in sorted((args.run_root/group).glob('episode_*')):
            if not (session/'telemetry.jsonl').exists():continue
            row,detail=audit_episode(session,reference,registration);output.append(row);details.append(detail)
    with (args.output_dir/'six_episodes.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(output[0]));writer.writeheader();writer.writerows(output)
    (args.output_dir/'audit.json').write_text(json.dumps(dict(thresholds=dict(xy_m=.25,yaw_rad=.25,max_deviation_m=.20),
       numeric_policy='float64 <=, no rounding or permissive epsilon; displayed rounded values never used for classification',
       nav2_succeeded=sum(r['nav2_succeeded'] for r in output),amcl_pass=sum(r['amcl_task_pass'] is True for r in output),
       gt_pass=sum(r['gt_task_pass'] is True for r in output),full_pass=sum(r['full_pass'] is True for r in output),
       episodes=output,details=details),indent=2))
    lines=['# Kiểm tra sáu episode đã ghi', '',
      'Giữ nguyên các log, model, Nav2 và ngưỡng. Chỉ tính lại, không chạy SAC.', '',
      '| Controller/lượt | NAV | AMCL xy / yaw | GT xy / yaw | Max ngang AMCL / GT | AMCL / GT / Full | Nguyên nhân |',
      '|---|---|---|---|---|---|---|']
    for r in output:
        lines.append(f"| {r['controller']}/{r['episode'][8:13]} | {r['outcome']} | {r['amcl_goal_xy_m']:.6f} m / {r['amcl_goal_yaw_rad']:.6f} rad | {r['gt_goal_xy_m']:.6f} m / {r['gt_goal_yaw_rad']:.6f} rad | {r['amcl_max_m']:.6f} / {r['gt_max_m']:.6f} m | {r['amcl_task_pass']} / {r['gt_task_pass']} / {r['full_pass']} | {r['failure_reasons']} |")
    lines += ['', 'Tất cả thời điểm max, timestamp nguồn và timestamp terminal có trong six_episodes.csv và audit.json.',
      'Đích được kiểm tra tại terminal sau STOPPING; không phải đúng chu kỳ FollowPath báo reached_goal.',
      'Max tính từ lúc logger bắt đầu sau FOLLOWING đến terminal; không chứa reset. Các max cũ đều xuất hiện trước vùng dừng cuối, xem timestamp.',
      'AMCL ở đây là TF map→base_footprint kết hợp AMCL và odometry, không phải message /amcl_pose đơn lẻ.',
      'Dữ liệu pose cuối không đổi giữa mẫu 50 Hz cuối và terminal; độ lệch GT không do lấy nhầm mẫu sau reset.',
      'Log cũ không có quaternion gốc, state từng mẫu hoặc timestamp chuẩn Nav2 reached_goal; không thể khôi phục các trường đó từ yaw. Logger mới bổ sung thụ động các bằng chứng này.',
      'Không fit GT vào AMCL và không sửa pose GT. Identity map/world là hợp đồng tọa độ danh định; sai số bản đồ/định vị được giữ là sai số thật của hệ thống.', '']
    (args.output_dir/'SIX_EPISODES.md').write_text('\n'.join(lines))
    print(json.dumps(dict(episodes=len(output),nav2_success=sum(r['nav2_succeeded'] for r in output),
       amcl_pass=sum(r['amcl_task_pass'] is True for r in output),gt_pass=sum(r['gt_task_pass'] is True for r in output),
       full_pass=sum(r['full_pass'] is True for r in output),output=str(args.output_dir))))


if __name__=='__main__':main()

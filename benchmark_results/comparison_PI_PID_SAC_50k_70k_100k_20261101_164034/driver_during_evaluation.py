#!/usr/bin/env python3
"""Chạy PI/PID và SAC 50k/70k/100k qua runner hiện có, xuất báo cáo offline."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import html
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from rl_training import ablation_suite as suite
from rl_training.experiment_guard import active_jobs, ResourceGuard
import benchmark_pi_pid_sac as metrics

SOURCES = {
    50000: 'train_20261027_135837_606611',
    70000: 'train_20261027_173345_545739',
    100000: 'train_20261101_115053_102278',
}
COLORS = ['#2164aa', '#db841b', '#159078', '#9859bb', '#d64d64']


def plans(episodes, seed, device):
    """Chốt mọi checkpoint/tham chiếu trước khi mở phiên mô phỏng đầu tiên."""
    common = dict(config=str(ROOT/'rl_training/config/sac.yaml'), purpose='evaluation',
        max_episode_seconds=None, gui=False, episodes=episodes, seed=seed, device=device,
        pid_config=str(ROOT/'rl_training/config/pid.yaml'), reference_path=None,
        map=str(ROOT/'maps/hall_01.yaml'), nav2_config=str(ROOT/'src/nino_navigation/config/nav2.yaml'))
    result = [('baselines', suite.make_plan(SimpleNamespace(**common, modes=['PI', 'PID'], sac_checkpoint=None)))]
    for steps, name in SOURCES.items():
        plan = suite.make_plan(SimpleNamespace(**common, modes=['SAC'],
            sac_checkpoint=str(ROOT/'rl_training/runs'/name/'model.zip')))
        if plan['groups'][0]['checkpoint']['timesteps'] != steps:
            raise ValueError(f'Checkpoint {name} không đúng {steps} step')
        result.append((f'sac_{steps}', plan))
    if len({p['reference_geometry_sha256'] for _, p in result}) != 1:
        raise ValueError('Các kế hoạch không dùng cùng đường tham chiếu')
    return result


def signature(group):
    env = deepcopy(group['config']['environment'])
    for key in ('effort_drive_config', 'benchmark_controller', 'resource_guard',
                'nav2_params_file', 'reference_path_file'):
        env.pop(key, None)
    control = {k: v for k, v in group['controller'].items() if k not in suite.PI_GAINS}
    return json.dumps([env, control, group['metadata'].get('nav2_snapshot_sha256'),
        group['metadata'].get('map'), group['reference']], sort_keys=True)


def read_groups(output):
    requests = [('PI', 'baselines/PI', 'PI'), ('PID', 'baselines/PID', 'PID')]
    requests += [(f'PI + SAC {steps//1000}k', f'sac_{steps}/PI_SAC', 'PI_SAC') for steps in SOURCES]
    groups = []
    for label, rel, kind in requests:
        folder = output/rel
        if not (folder/'metadata.json').exists():
            continue
        group = metrics.load_group(folder, kind, [], 0.5)
        if kind == 'PI_SAC':
            expected = int(rel.split('/')[0].split('_')[1])
            if not group['checkpoint_proof']['verified'] or group['checkpoint_proof']['timesteps'] != expected:
                raise ValueError('Checkpoint/config không xác minh được ở '+str(folder))
        group['display_label'] = label
        groups.append(group)
    return groups


def summary_cells(groups, key, comparable=True):
    values = [g['summary'][key] for g in groups]
    direction = metrics.METRICS[key][3]
    valid = [s['mean'] for s in values if s['mean'] is not None]
    best = (min(valid) if direction == 'min' else max(valid)) if valid and direction and comparable else None
    result = []
    for value in values:
        text = html.escape(metrics.format_stats(value))
        if best is not None and value['mean'] is not None and math.isclose(value['mean'], best, rel_tol=1e-9, abs_tol=1e-12):
            text = '<b>'+text+'</b>'
        detail = ('Median %.4g · P95 %.4g · n=%d' % (value['median'], value['p95'], value['n'])) if value['n'] else 'N/A: xem lý do thiếu dữ liệu'
        result.append('<td>'+text+'<small>'+html.escape(detail)+'</small></td>')
    return ''.join(result)


def write_report(output, episodes):
    groups = read_groups(output)
    if not groups:
        raise ValueError('Chưa có metadata evaluation để tạo báo cáo')
    complete = len(groups) == 5 and all(g['complete_count'] == episodes for g in groups)
    comparable = complete and len({signature(g) for g in groups}) == 1
    notices = [
        f'Mỗi nhóm dự kiến {episodes} lượt. '+('Đã đủ mọi nhóm.' if complete else 'BÁO CÁO CHƯA ĐỦ LƯỢT: xem trạng thái runner.'),
        'Nguồn pose là TF map→base_footprint do AMCL + odometry encoder cung cấp; không phải vị trí thực tuyệt đối.',
        'Nav2 SUCCEEDED và task pass AMCL được ghi riêng. Task pass: xy≤0.25 m, |yaw|≤0.25 rad, max lệch ngang≤0.20 m.',
        'PI và PI nền SAC giữ Kp=.30, Ki=.10, Kd=0. PID dùng profile riêng, không tuning trong phép so sánh này.',
        'SAC chỉ predict deterministic; không học thêm. Cùng map, spawn, goal, reference, Nav2 và giới hạn chấp hành.',
        'Seed 1/2/3 là nhãn Gym reset; không chứng minh cùng nhiễu vật lý Gazebo hoặc AMCL.',
        'Vận tốc thân từ encoder odometry là ước lượng; không chứng minh không trượt. Mô-men là lệnh sau limiter, không phải đo lực.',
        'RMSE/P95 trong episode dùng trọng số timestamp nguồn. Mean ± SD (SD mẫu), Median/P95 ở bảng là phân bố giữa các episode.',
        'Giữ mọi thất bại điều khiển. Startup/worker lỗi được công khai riêng, không cộng thành lượt điều khiển thất bại hoặc xóa log.',
        'Chỉ 3–4 lượt/nhóm là khảo sát nhỏ, chưa đủ để kết luận độ tin cậy thống kê hoặc khả năng tổng quát.',
        'Không tính cửa sổ quanh gờ vì chưa có vị trí gờ đã xác minh trong map; không lấy tọa độ world gán trực tiếp cho AMCL.',
    ]
    if not comparable:
        notices.append('CONFOUND/INCOMPLETE: chưa xác minh đủ cùng điều kiện và số lượt; không tô giá trị tốt nhất.')
    if any(any(e['coverage']['amcl'] < 1 for e in g['episodes']) for g in groups):
        notices.append('Một số lượt không đủ coverage AMCL; kết luận task pass có thể N/A. Xem audit và lý do từng lượt.')
    summary, ep_rows = [], []
    for g in groups:
        for key, value in g['summary'].items():
            section, name, unit, direction = metrics.METRICS[key]
            summary.append(dict(group=g['display_label'], metric=key, section=section, description=name,
                unit=unit, direction=direction, **value))
        for e in g['episodes']:
            ep_rows.append(dict(group=g['display_label'], episode=e['episode'], outcome=e['outcome'],
                duration_s=e['duration_s'], mission_failure_reasons=e['mission_failure_reasons'],
                source=e['source'], **e['metrics'], **{'coverage_'+k:v for k,v in e['coverage'].items()}))
    metrics.write_csv(output/'summary.csv', summary)
    metrics.write_csv(output/'episodes.csv', ep_rows)
    payload = metrics.clean(dict(created_utc=datetime.now(timezone.utc).isoformat(), complete=complete,
        comparable=comparable, groups=groups, notices=notices, expected_episodes=episodes,
        report_source_sha256=suite.sha(Path(__file__)), metric_source_sha256=suite.sha(ROOT/'benchmark_pi_pid_sac.py'),
        definitions={k:dict(section=v[0],name=v[1],unit=v[2],direction=v[3]) for k,v in metrics.METRICS.items()}))
    suite.save_json(output/'report_data.json', payload)
    head = '<tr><th>Chỉ số</th>'+''.join('<th>'+html.escape(g['display_label'])+'</th>' for g in groups)+'</tr>'
    rows = []
    for key, (section, name, unit, _) in metrics.METRICS.items():
        rows.append('<tr><th>'+html.escape(f'{section} · {name} ({unit})')+'</th>'+summary_cells(groups,key,comparable)+'</tr>')
    table_episodes = []
    for g in groups:
        for e in g['episodes']:
            vals=[g['display_label'],e['episode'],e['outcome'],e['metrics']['mission_success_pct'],
                e['metrics']['amcl_goal_xy_m'],e['metrics']['amcl_goal_yaw_rad'],e['metrics']['amcl_max_m'],
                e['duration_s'],'; '.join(e['mission_failure_reasons']) or 'Đạt các điều kiện đã kiểm tra']
            table_episodes.append('<tr>'+''.join('<td>'+html.escape('N/A' if v is None else f'{v:.5g}' if isinstance(v,float) else str(v))+'</td>' for v in vals)+'</tr>')
    cards=[]
    for i,g in enumerate(groups):
        known=[e for e in g['episodes'] if e['outcome']!='INCOMPLETE']
        nav=sum(e['metrics']['nav_success_pct']==100 for e in known)
        task=sum(e['metrics']['mission_success_pct']==100 for e in known)
        unknown=sum(e['metrics']['mission_success_pct'] is None for e in known)
        proof=g['checkpoint_proof']
        cards.append(f'<div class="card" style="border-top-color:{COLORS[i]}"><h3>'+html.escape(g['display_label'])+
            f'</h3><b>{task}/{len(known)} task pass AMCL</b><p>Nav2: {nav}/{len(known)} · Task chưa đủ dữ liệu: {unknown}</p><small>'+
            ('Checkpoint '+str(proof['timesteps'])+' step, SHA/config đã kiểm tra' if proof else 'SAC tắt')+'</small></div>')
    traces=[dict(label=g['display_label'],color=COLORS[i],reference=g['reference'],
        traces=[dict(episode=e['episode'],outcome=e['outcome'],**e['trace']) for e in g['episodes'] if e['outcome']!='INCOMPLETE']) for i,g in enumerate(groups)]
    charts_json=json.dumps(metrics.clean(traces),ensure_ascii=False,separators=(',',':'),allow_nan=False).replace('<',chr(92)+'u003c')
    audit=html.escape(json.dumps(metrics.clean({g['display_label']:dict(path=g['path'],gains=g['metadata'].get('gains'),
        checkpoint=g['checkpoint_proof'],events=g['audit'],episodes=[dict(episode=e['episode'],coverage=e['coverage'],
        missing_reasons=e['missing_reasons'],source_sha256=e['source_sha256']) for e in g['episodes']]) for g in groups}),ensure_ascii=False,indent=2))
    page = HTML.replace('@@CARDS@@',''.join(cards)).replace('@@NOTICES@@',''.join('<li>'+html.escape(n)+'</li>' for n in notices))
    page=page.replace('@@TABLE@@',head+''.join(rows)).replace('@@EPISODES@@',''.join(table_episodes))
    page=page.replace('@@AUDIT@@',audit).replace('@@CHARTS@@',charts_json).replace('@@DATE@@',html.escape(payload['created_utc']))
    (output/'report_PI_PID_SAC_50k_70k_100k.html').write_text(page,encoding='utf-8')
    return payload


HTML = r'''<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>PI · PID · SAC 50k / 70k / 100k</title><style>
*{box-sizing:border-box}body{font:15px system-ui,sans-serif;margin:0;background:#eef3f7;color:#193048}header{padding:35px 4vw;background:#102b43;color:white}header p{color:#cce0ef}h1{font-size:32px}main{max-width:1700px;margin:auto;padding:20px 3vw}.cards{display:grid;grid-template-columns:repeat(5,1fr);gap:12px}.card{border-top:4px solid;background:white;padding:15px;border-radius:7px}.card h3{margin:0 0 12px}.card b{font-size:19px}.card p,.card small{font-size:12px;line-height:1.7}section{background:white;padding:20px;border-radius:9px;margin:18px 0;border:1px solid #dce5ed}h2{font-size:21px}p,li{line-height:1.7}li{margin:5px 0}.scroll{overflow:auto;max-height:75vh}table{border-collapse:collapse;width:100%;min-width:1150px;font-size:13px}th,td{padding:12px;border-bottom:1px solid #dde5ec;text-align:left;vertical-align:top}th{background:#eaf1f7;position:sticky;top:0}tr:nth-child(even){background:#f8fafc}td small{display:block;color:#63778c;margin-top:5px}td b{color:#076c56}select,button{padding:9px;border:1px solid #abc1d2;border-radius:5px;background:white;font:inherit}.plots{display:grid;grid-template-columns:1fr 1fr;gap:14px}.plot{border:1px solid #dce5ed;border-radius:6px;padding:12px}.plot h3{font-size:15px;margin:4px}svg{width:100%;height:auto}.legend{display:flex;flex-wrap:wrap;gap:15px;margin:15px 0}.legend span{font-weight:700;font-size:13px}.note{color:#63778c;font-size:13px}pre{white-space:pre-wrap;overflow-wrap:anywhere;max-height:650px;overflow:auto;background:#f1f5f8;padding:15px;font-size:12px}a{color:#126d90}noscript{display:block;padding:20px;background:#ffe9b0}@media(max-width:1050px){.cards{grid-template-columns:repeat(2,1fr)}.plots{grid-template-columns:1fr}}@media print{.scroll{max-height:none;overflow:visible}th{position:static}select,button{display:none}}
</style></head><body><header><h1>PI · PID · PI + SAC 50k / 70k / 100k</h1><p>So sánh theo AMCL + IMU + encoder · Evaluation deterministic · Báo cáo offline</p><small>Xuất: @@DATE@@</small></header><main><noscript>Bật JavaScript để xem biểu đồ SVG. Bảng chỉ số và nội dung báo cáo vẫn đọc được offline.</noscript><div class="cards">@@CARDS@@</div><section><h2>Điều kiện và giới hạn diễn giải</h2><ul>@@NOTICES@@</ul><p><a href="summary.csv">Tải bảng tổng hợp CSV</a> · <a href="episodes.csv">CSV từng episode</a> · <a href="protocol.json">Protocol và kế hoạch</a> · <a href="report_data.json">Dữ liệu/bằng chứng đầy đủ</a> · <a href="status.json">Trạng thái job</a></p></section><section><h2>Bảng so sánh 5 nhóm</h2><p class="note">Mỗi ô: Mean ± SD mẫu; bên dưới là Median, P95 giữa các episode và số lượt có dữ liệu. Giá trị nhỏ nhất/lớn nhất chỉ được tô khi có chiều ưu tiên rõ và cùng điều kiện; không tô tỷ lệ đóng góp SAC hay tốc độ như một điểm tốt/xấu.</p><div class="scroll"><table id="metrics-table">@@TABLE@@</table></div></section><section><h2>Đường đi, rung và mô-men theo tiến độ</h2><label>Chọn lượt: <select id="episode"></select></label><div class="legend" id="legend"></div><p class="note">Các lượt cùng số không phải cùng nhiễu vật lý. AMCL là định vị ước lượng. Mô-men trái nét liền, phải nét đứt; đường tham chiếu màu xám. Khi vẽ giữ tối đa khoảng 1.800 mẫu/đường, còn metric dùng đầy đủ mẫu nguồn. Không vẽ nối qua dữ liệu thiếu.</p><div class="plots"><div class="plot"><h3>Quỹ đạo AMCL: X–Y (m)</h3><svg id="trajectory" viewBox="0 0 720 300"></svg></div><div class="plot"><h3>Lỗi ngang e_y theo s (cm)</h3><svg id="error" viewBox="0 0 720 300"></svg></div><div class="plot"><h3>Gia tốc thân a_z đã trừ trọng lực theo s (m/s²)</h3><svg id="az" viewBox="0 0 720 300"></svg></div><div class="plot"><h3>Jerk z lọc 5 Hz theo s (m/s³)</h3><svg id="jerk" viewBox="0 0 720 300"></svg></div><div class="plot"><h3>Lệnh mô-men hai bánh sau limiter theo s (N·m)</h3><svg id="torque" viewBox="0 0 720 300"></svg></div></div></section><section><h2>Mọi lượt thử và lý do task pass/fail</h2><div class="scroll"><table><thead><tr><th>Nhóm</th><th>Episode</th><th>Nav2 outcome</th><th>Task AMCL (%)</th><th>Lỗi đích XY (m)</th><th>Lỗi yaw (rad)</th><th>Max lệch (m)</th><th>Thời gian quan sát (s)</th><th>Lý do</th></tr></thead><tbody>@@EPISODES@@</tbody></table></div></section><section><h2>Phương pháp đo</h2><p>RMSE = √(Σe²Δt / ΣΔt). Pose AMCL và terminal dùng timestamp nguồn, không ngoại suy cuối lượt, không lấy pose có sai số nhỏ nhất để thay terminal. Không nối qua gap nguồn lớn hơn 0,10 s. Gia tốc body đã trừ vector trọng lực 9,80665 m/s² theo quaternion IMU. Jerk là sai phân a_z theo timestamp IMU và lọc nhân quả bậc một 5 Hz. Wheel RMSE theo mục tiêu và vận tốc đo JointState; limiter theo counter vòng điều khiển khi có. Thời gian hoàn thành chỉ tính lượt đạt task pass; thời gian quan sát vẫn ghi cho mọi lượt.</p><details><summary>Nguồn checkpoint, gain, lỗi hạ tầng và dữ liệu thiếu</summary><pre>@@AUDIT@@</pre></details></section></main><script id="chart-data" type="application/json">@@CHARTS@@</script><script>
'use strict';const groups=JSON.parse(document.getElementById('chart-data').textContent);const ns='http://www.w3.org/2000/svg';function el(t,a={},text){const n=document.createElementNS(ns,t);for(const[k,v]of Object.entries(a))n.setAttribute(k,v);if(text!==undefined)n.textContent=text;return n}function draw(id,series,equal=false){const svg=document.getElementById(id);svg.replaceChildren();let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(const s of series)for(const[x,y]of s.points)if(Number.isFinite(x)&&Number.isFinite(y)){xmin=Math.min(xmin,x);xmax=Math.max(xmax,x);ymin=Math.min(ymin,y);ymax=Math.max(ymax,y)}if(!Number.isFinite(xmin)){svg.append(el('text',{x:30,y:80},'N/A: không có dữ liệu hợp lệ'));return}if(xmax===xmin)xmax=xmin+1;if(ymax===ymin){ymin-=.1;ymax+=.1}let pad=(ymax-ymin)*.12;ymin-=pad;ymax+=pad;if(equal){const c=(ymax+ymin)/2,span=Math.max(ymax-ymin,(xmax-xmin)*215/620);ymin=c-span/2;ymax=c+span/2}const X=x=>65+(x-xmin)/(xmax-xmin)*620,Y=y=>250-(y-ymin)/(ymax-ymin)*215;for(let i=0;i<=5;i++){const x=xmin+(xmax-xmin)*i/5,y=ymin+(ymax-ymin)*i/5;svg.append(el('line',{x1:65,x2:685,y1:Y(y),y2:Y(y),stroke:'#e0e7ef'}),el('text',{x:6,y:Y(y)+4,'font-size':11},y.toFixed(3)),el('text',{x:X(x)-7,y:275,'font-size':11},x.toFixed(2)))}for(const s of series){let points=[];function flush(){if(points.length>1)svg.append(el('polyline',{points:points.join(' '),fill:'none',stroke:s.color,'stroke-width':1.4,'stroke-dasharray':s.dash?'5 4':''}));points=[]}const step=Math.max(1,Math.ceil(s.points.length/1800));for(let i=0;i<s.points.length;i++){const[x,y]=s.points[i];if(!Number.isFinite(x)||!Number.isFinite(y)){flush();continue}if(i%step===0||i===s.points.length-1)points.push(X(x)+','+Y(y))}flush()}}
function render(){const i=Number(document.getElementById('episode').value),p={trajectory:[],error:[],az:[],jerk:[],torque:[]};for(const g of groups){const t=g.traces[i];if(!t)continue;const add=(key,x,y,dash=false)=>p[key].push({color:g.color,dash,points:x.map((v,j)=>[v,y[j]])});add('trajectory',t.x,t.y);add('error',t.s,t.error.map(v=>v===null?null:v*100));add('az',t.s,t.az);add('jerk',t.s,t.jerk);add('torque',t.s,t.torque_left);add('torque',t.s,t.torque_right,true)}const ref=groups[0]?.reference?.poses;if(ref)p.trajectory.unshift({color:'#586575',dash:true,points:ref.map(v=>[v.x,v.y])});for(const[key,series]of Object.entries(p))draw(key,series,key==='trajectory');document.body.dataset.renderedEpisode=i+1}const menu=document.getElementById('episode');for(let i=0;i<Math.max(0,...groups.map(g=>g.traces.length));i++){const o=document.createElement('option');o.value=i;o.textContent='Lượt '+(i+1);menu.append(o)}for(const g of groups){const n=document.createElement('span');n.style.color=g.color;n.textContent=g.label;document.getElementById('legend').append(n)}menu.onchange=render;render();document.body.dataset.groups=groups.length;
</script></body></html>'''


def run(args):
    output = args.output_root.resolve()
    if args.report_only:
        return write_report(output, args.episodes)
    if active_jobs() or suite.active_gazebo():
        raise ValueError('Đang có job hoặc Gazebo; không chạy chồng')
    ResourceGuard(ROOT).check(force=True)
    prepared = plans(args.episodes, args.seed, args.device)
    protocol = dict(episodes=args.episodes, seed=args.seed, device=args.device, plans=prepared,
        driver_sha256=suite.sha(Path(__file__)), controller_count=5, deterministic=True,
        no_training=True, no_tuning=True, maximum_suite_attempts=2,
        retry_policy='Only infrastructure failures; preserve all attempts; never retry control failure',
        created_utc=datetime.now(timezone.utc).isoformat())
    if args.dry_run:
        print(json.dumps(protocol, ensure_ascii=False, indent=2));return protocol
    output.mkdir(parents=True, exist_ok=args.resume)
    if args.resume:
        stored = json.loads((output/'protocol.json').read_text())
        for key in ('plans','episodes','seed','device','driver_sha256'):
            if stored[key] != protocol[key]:
                raise ValueError('Resume khác protocol: '+key)
    else:
        suite.save_json(output/'protocol.json', protocol)
    state = dict(state='RUNNING', completed=[], current=None, attempts=[])
    if args.resume and (output/'status.json').exists():
        state = json.loads((output/'status.json').read_text());state['state']='RUNNING'
    def save():
        state['updated_utc']=datetime.now(timezone.utc).isoformat();suite.save_json(output/'status.json',state)
    save()
    with suite.machine_lock():
        try:
            for name, plan in prepared:
                state['current']=name;save()
                if name in state['completed']:
                    continue
                destination=output/name
                code=2
                for attempt in range(2):
                    ResourceGuard(ROOT).check(force=True)
                    print(f'RUN {name}, attempt {attempt+1}, episodes={args.episodes}',flush=True)
                    code=suite.execute(plan,destination,resume=destination.exists())
                    state['attempts'].append(dict(group=name,attempt=attempt+1,exit_code=code,utc=datetime.now(timezone.utc).isoformat()));save()
                    if code==0 or code==130:
                        break
                    specs=list(destination.glob('*/infrastructure_fault.json'))
                    if not specs or any(json.loads(p.read_text()).get('type')=='ResourceStop' for p in specs):
                        break
                    if suite.active_gazebo():
                        break
                    time.sleep(3)
                if code:
                    state.update(state='INTERRUPTED' if code==130 else 'FAILED',exit_code=code);save()
                    write_report(output,args.episodes)
                    return state
                state['completed'].append(name);save()
            report=write_report(output,args.episodes)
            state.update(state='COMPLETE' if report['complete'] else 'INCOMPLETE',current=None,
                         report=str(output/'report_PI_PID_SAC_50k_70k_100k.html'));save()
            print(json.dumps(state,ensure_ascii=False),flush=True)
            return state
        except KeyboardInterrupt:
            state['state']='INTERRUPTED';save();raise
        except Exception as exc:
            state.update(state='FAILED',error=str(exc));save();raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',required=True,type=Path)
    parser.add_argument('--episodes',type=int,choices=[3,4],default=3)
    parser.add_argument('--seed',type=int,default=1)
    parser.add_argument('--device',choices=['cpu','cuda','auto'],default='cuda')
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--report-only',action='store_true')
    parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    result=run(args)
    if result.get('state') in ('FAILED','INTERRUPTED','INCOMPLETE'):
        return 2
    return 0


if __name__=='__main__':
    raise SystemExit(main())

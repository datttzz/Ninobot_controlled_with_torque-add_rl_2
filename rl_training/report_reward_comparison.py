#!/usr/bin/env python3
"""Đọc log reward hiện có và xuất HTML/SVG/CSV; không train hoặc mở ROS."""
import argparse
import csv
from datetime import datetime
import hashlib
import html
import json
import math
from pathlib import Path
import statistics
import sys
import zipfile
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'rl_training'))
from plotting_compat import prepare_matplotlib_namespace
prepare_matplotlib_namespace()
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np
import yaml

KEYS = ['progress', 'cross_track', 'heading', 'vibration', 'effort', 'smoothness', 'time', 'terminal']
NAMES = dict(progress='Thưởng tiến độ', cross_track='Phạt lệch ngang', heading='Phạt lệch hướng',
    vibration='Phạt rung (gia tốc 3 trục)', effort='Phạt độ lớn action SAC',
    smoothness='Phạt thay đổi action SAC', time='Phạt thời gian', terminal='Thưởng/phạt kết thúc')
TRAIN = {50000:'train_20261027_135837_606611', 70000:'train_20261027_173345_545739',
         100000:'train_20261101_115053_102278'}
EVAL_ROOT = ROOT/'benchmark_results/comparison_PI_PID_SAC_50k_70k_100k_20261101_164034'
EVALUATION = [('50k',50000,EVAL_ROOT/'sac_50000/PI_SAC'),
    ('70k',70000,EVAL_ROOT/'sac_70000/PI_SAC'),
    ('100k đợt đầu',100000,EVAL_ROOT/'sac_100000/PI_SAC'),
    ('100k chạy lại',100000,ROOT/'rl_training/runs/sac100k_rerun_20261101_171250/evaluation/PI_SAC')]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def reduce_terms(rows):
    if not rows:
        raise ValueError('Episode không có bước reward')
    for row in rows:
        terms = row.get('reward_terms')
        if not isinstance(terms,dict) or set(terms) != set(KEYS):
            raise ValueError('Thiếu hoặc khác schema reward_terms; không thay bằng 0')
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in terms.values()):
            raise ValueError('Reward không hữu hạn hoặc sai kiểu')
    return {k:math.fsum(row['reward_terms'][k] for row in rows) for k in KEYS}


def episode(path):
    rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    sums=reduce_terms(rows)
    return dict(episode=path.parent.name,source=str(path),source_sha256=sha(path),
        steps=len(rows),duration_s=rows[-1].get('simulation_time'),
        outcome=rows[-1].get('outcome'),terms=sums,total=math.fsum(sums.values()))


def stats(values):
    if not values:
        raise ValueError('Không có dữ liệu để tổng hợp')
    return dict(n=len(values),mean=statistics.mean(values),
        std=statistics.stdev(values) if len(values)>1 else None,
        median=statistics.median(values),minimum=min(values),maximum=max(values))


def group(label,kind,records,**evidence):
    if not records or any(e['outcome'] not in ('SUCCEEDED','FAILED','TIME_LIMIT') for e in records):
        raise ValueError('Chỉ tổng hợp episode đã kết thúc; không âm thầm bỏ lượt thất bại')
    return dict(label=label,kind=kind,episodes=records,total=stats([e['total'] for e in records]),
        components={k:stats([e['terms'][k] for e in records]) for k in KEYS},
        nav_successes=sum(e['outcome']=='SUCCEEDED' for e in records),**evidence)


def checkpoint(path,expected):
    with zipfile.ZipFile(path) as z:
        data=json.loads(z.read('data'))
    if data['num_timesteps'] != expected or 'ent_coef' not in data:
        raise ValueError('Checkpoint không đúng SAC/step: '+str(path))
    return dict(path=str(path),sha256=sha(path),timesteps=data['num_timesteps'],
                stats_window_size=data.get('_stats_window_size'))


def collect(window):
    training,evaluation,weights,checks=[],[],[],[]
    for steps,name in TRAIN.items():
        folder=ROOT/'rl_training/runs'/name
        proof=checkpoint(folder/'model.zip',steps)
        config=folder/'config.yaml';weights.append(yaml.safe_load(config.read_text())['reward'])
        with (folder/'monitor.csv').open() as f:
            next(f);monitor=list(csv.DictReader(f))
        selected=[]
        for ep in sorted(folder.glob('episode_*'),reverse=True):
            p=ep/'steps.jsonl'
            if not p.is_file() or not p.stat().st_size:
                continue
            # Xem terminal trước, tránh nạp mọi log trong toàn bộ run.
            with p.open('rb') as f:
                f.seek(max(0,p.stat().st_size-20000));tail=f.read().decode().splitlines()
            last=json.loads(tail[-1])
            if last.get('outcome') not in ('SUCCEEDED','FAILED','TIME_LIMIT'):
                continue
            selected.append(episode(p))
            if len(selected)==window:
                break
        selected.reverse()
        if len(selected)!=window or len(monitor)<window:
            raise ValueError('Không đủ episode hoàn tất trong '+str(folder))
        for e,row in zip(selected,monitor[-window:]):
            if abs(e['total']-float(row['r']))>1e-5 or e['steps']!=int(row['l']):
                raise ValueError('Tổng reward_terms không khớp Monitor: '+e['source'])
        training.append(group(f'{steps//1000}k','train_last_window',selected,checkpoint=proof,
            source_run=str(folder),config_sha256=sha(config),monitor_sha256=sha(folder/'monitor.csv')))
        checks.append(dict(check='monitor_matches_reward_terms',group=f'{steps//1000}k',episodes=window,tolerance=1e-5))
    signatures=[]
    for label,steps,folder in EVALUATION:
        meta=json.loads((folder/'metadata.json').read_text())
        if meta.get('mode')!='evaluate' or meta.get('deterministic') is not True:
            raise ValueError('Nguồn evaluate không deterministic: '+str(folder))
        proof=checkpoint(Path(meta['model']),steps)
        if proof['sha256']!=meta['checkpoint_evidence']['sha256']:
            raise ValueError('Hash model khác bằng chứng evaluate')
        for filename,key in [('config.yaml','config_snapshot_sha256'),('controller.yaml','controller_snapshot_sha256')]:
            if sha(folder/filename)!=meta[key]:
                raise ValueError('Snapshot evaluate đã đổi: '+str(folder/filename))
        config=yaml.safe_load((folder/'config.yaml').read_text());weights.append(config['reward'])
        nav=Path(config['environment']['nav2_params_file'])
        if sha(nav)!=meta['nav2_snapshot_sha256']:
            raise ValueError('Nav2 snapshot đã đổi')
        environment=dict(config['environment'])
        for key in ('effort_drive_config','benchmark_controller','resource_guard','nav2_params_file','reference_path_file'):
            environment.pop(key,None)
        signatures.append(json.dumps([environment,meta['controller'],meta['reference_geometry_sha256'],meta['nav2_snapshot_sha256'],meta['map']],sort_keys=True))
        records=json.loads((folder/'result.json').read_text());selected=[]
        for row in records:
            e=episode(Path(row['session'])/'steps.jsonl')
            if abs(e['total']-row['reward'])>1e-7 or e['steps']!=row['steps'] or e['outcome']!=row['outcome']:
                raise ValueError('Reward/outcome không khớp result.json: '+e['source'])
            selected.append(e)
        if len(selected)!=3:
            raise ValueError('Phải có đúng 3 lượt trong đợt evaluate '+label)
        evaluation.append(group(label,'evaluate',selected,checkpoint=proof,source_run=str(folder),
            metadata_sha256=sha(folder/'metadata.json'),result_sha256=sha(folder/'result.json')))
        checks.append(dict(check='result_matches_reward_terms',group=label,episodes=3,tolerance=1e-7))
    if not all(w==weights[0] for w in weights):
        raise ValueError('Trọng số reward giữa các nguồn khác nhau')
    if len(set(signatures))!=1:
        raise ValueError('Các đợt evaluate khác điều kiện cấu hình/đường/giới hạn')
    combined=group('100k gộp cả 6 lượt','evaluate_derived',
        evaluation[2]['episodes']+evaluation[3]['episodes'],
        note='Tập dẫn xuất, trùng dữ liệu hai đợt 100k; không phải sáu lượt mới hoặc một nhóm độc lập.')
    return dict(training=training,evaluation=evaluation,combined_100k=combined,weights=weights[0],
                validation=checks,train_window=window)


def write_csv(path,rows):
    with path.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def figures(data,out):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    result=[]
    fig,ax=plt.subplots(figsize=(10,4.4),layout='constrained')
    for g,color in zip(data['training'],['#2874a6','#159179','#be615c']):
        ax.plot(range(1,len(g['episodes'])+1),[e['total'] for e in g['episodes']],marker='o',label=g['label'],color=color)
    ax.set(xlabel='Thứ tự episode trong cửa sổ cuối mỗi run (không ghép cặp nhiễu)',ylabel='Tổng reward / episode',title=f"{data['train_window']} episode hoàn tất cuối trước mỗi checkpoint")
    ax.grid(alpha=.2);ax.legend();result.append(('training_reward.svg',fig))
    fig,ax=plt.subplots(figsize=(10,4.4),layout='constrained')
    colors=['#2874a6','#159179','#9d6ab2','#be615c']
    for i,(g,color) in enumerate(zip(data['evaluation'],colors)):
        ax.errorbar(i,g['total']['mean'],yerr=g['total']['std'],fmt='s',capsize=7,color=color,markersize=8)
        for j,e in enumerate(g['episodes']):
            ax.scatter(i+(j-1)*.10,e['total'],color='#d62828' if e['outcome']!='SUCCEEDED' else color,s=40,zorder=3)
    ax.set_xticks(range(4),[g['label'] for g in data['evaluation']]);ax.set(ylabel='Tổng reward / episode',title='Evaluate: từng lượt và Mean ± SD mẫu; chấm đỏ là lượt thất bại')
    ax.grid(axis='y',alpha=.2);result.append(('evaluation_reward.svg',fig))
    fig,ax=plt.subplots(figsize=(10,4.8),layout='constrained');bottom=np.zeros(4)
    for key,color in zip(['vibration','cross_track','heading','effort','smoothness','time'],['#db8d3d','#2874a6','#9173be','#61a590','#d35d6e','#82919d']):
        values=np.array([-g['components'][key]['mean'] for g in data['evaluation']])
        ax.bar(range(4),values,bottom=bottom,label=NAMES[key],color=color);bottom+=values
    ax.set_xticks(range(4),[g['label'] for g in data['evaluation']]);ax.set(ylabel='Độ lớn điểm phạt trung bình / episode',title='Tách các điểm phạt; không gồm thưởng tiến độ và terminal')
    ax.legend(loc='upper left',bbox_to_anchor=(1,1),fontsize=8);ax.grid(axis='y',alpha=.2);result.append(('reward_penalties.svg',fig))
    snippets=[]
    for name,fig in result:
        path=out/name;fig.savefig(path,format='svg');plt.close(fig)
        svg=path.read_text();snippets.append(svg[svg.index('<svg'):])
    return snippets


def fmt(value):
    if value['std'] is None:
        return f"{value['mean']:.2f} ± N/A"
    return f"{value['mean']:.2f} ± {value['std']:.2f}"


def component_table(groups):
    head='<tr><th>Thành phần (điểm / episode)</th>'+''.join('<th>'+html.escape(g['label'])+f'<small>n={g["total"]["n"]}</small></th>' for g in groups)+'</tr>'
    body=[]
    for key in KEYS:
        body.append('<tr><th>'+html.escape(NAMES[key])+'</th>'+''.join(f'<td>{g["components"][key]["mean"]:+.3f}</td>' for g in groups)+'</tr>')
    body.append('<tr class="total"><th>Reward tổng: Mean ± SD</th>'+''.join('<td>'+fmt(g['total'])+'</td>' for g in groups)+'</tr>')
    return '<div class="scroll"><table><thead>'+head+'</thead><tbody>'+''.join(body)+'</tbody></table></div>'


def render(data,out,plots):
    eval_groups=data['evaluation'];combined=data['combined_100k']
    cards=[]
    for g in [*eval_groups[:2],eval_groups[3],combined]:
        cards.append('<div class="card"><span>'+html.escape(g['label'])+'</span><b>'+fmt(g['total'])+
            '</b><small>'+f'n={g["total"]["n"]} · Nav2 thành công {g["nav_successes"]}/{g["total"]["n"]}'+'</small></div>')
    rows=[]
    for g in [*data['training'],*eval_groups]:
        for i,e in enumerate(g['episodes'],1):
            rows.append('<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in [g['kind'],g['label'],i,e['episode'],e['outcome'],f'{e["total"]:.3f}',e['steps']])+'</tr>')
    weights=html.escape(json.dumps(data['weights'],ensure_ascii=False,indent=2))
    provenance=html.escape(json.dumps(dict(training=[dict(label=g['label'],source_run=g['source_run'],checkpoint=g['checkpoint'],episodes=[dict(source=e['source'],sha256=e['source_sha256']) for e in g['episodes']]) for g in data['training']],
        evaluation=[dict(label=g['label'],source_run=g['source_run'],checkpoint=g['checkpoint'],episodes=[dict(source=e['source'],sha256=e['source_sha256']) for e in g['episodes']]) for g in eval_groups],validation=data['validation']),ensure_ascii=False,indent=2))
    best=max(eval_groups,key=lambda g:g['total']['mean'])
    conclusion=(f'Trong log evaluate này, {best["label"]} có reward trung bình cao nhất ({best["total"]["mean"]:.2f}). '
        f'100k đợt đầu: {fmt(eval_groups[2]["total"])}; đợt chạy lại: {fmt(eval_groups[3]["total"])}; '
        f'gộp đủ 6 lượt: {fmt(combined["total"])}. Hai đợt dùng cùng model; khác kết quả không có nghĩa model đã học thêm.')
    page=PAGE.replace('@@DATE@@',html.escape(data['created_local'])).replace('@@CARDS@@',''.join(cards))
    page=page.replace('@@CONCLUSION@@',html.escape(conclusion)).replace('@@TRAIN@@',component_table(data['training']))
    page=page.replace('@@EVAL@@',component_table(eval_groups+[combined])).replace('@@WINDOW@@',str(data['train_window']))
    for i,svg in enumerate(plots):page=page.replace(f'@@PLOT{i}@@',svg)
    page=page.replace('@@ROWS@@',''.join(rows)).replace('@@WEIGHTS@@',weights).replace('@@PROVENANCE@@',provenance)
    (out/'reward_comparison.html').write_text(page,encoding='utf-8')


PAGE='''<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Reward SAC 50k · 70k · 100k</title><style>
*{box-sizing:border-box}body{font:15px system-ui,sans-serif;color:#193148;background:#edf3f7;margin:0}header{padding:32px 4vw;background:#102b43;color:white}header p{color:#cfe1ef}h1{font-size:32px;margin:8px 0 14px}main{max-width:1550px;padding:24px 3vw;margin:auto}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px}.card{background:white;padding:18px;border-radius:8px;border-top:4px solid #258b94}.card span{font-size:14px}.card b{display:block;font-size:24px;margin:12px 0}.card small,small{display:block;color:#63798c;margin-top:5px}section{background:white;border:1px solid #d9e5ed;border-radius:9px;margin:18px 0;padding:22px}h2{font-size:22px;margin:4px 0 14px}p,li{line-height:1.75}li{margin:6px 0}.notice{padding:15px;background:#eaf5f2;border-left:4px solid #258b94}.warning{background:#fff6e5;border-left:4px solid #d7a241;padding:12px}.scroll{overflow:auto;max-height:650px}table{border-collapse:collapse;width:100%;min-width:900px;font-size:13px}th,td{padding:12px;border-bottom:1px solid #dce7ee;text-align:left;vertical-align:top}th{background:#eaf1f6}thead th{position:sticky;top:0}tr:nth-child(even){background:#f8fbfd}.total{font-weight:bold;color:#075b65;background:#e8f4f3!important}.plot svg{width:100%;height:auto;max-height:490px}a{color:#126e8c}.note{font-size:13px;color:#5d7387}code,pre{font-family:ui-monospace,monospace}pre{background:#f0f5f8;padding:15px;overflow:auto;max-height:500px;font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere}summary{cursor:pointer;color:#136b85;font-weight:600}.grid{display:grid;grid-template-columns:1fr 1fr;gap:15px}.grid .plot{min-width:0}@media(max-width:950px){.cards{grid-template-columns:repeat(2,1fr)}.grid{grid-template-columns:1fr}}@media print{.scroll{max-height:none;overflow:visible}thead th{position:static}section{break-inside:avoid}}
</style></head><body><header><h1>Reward SAC: 50k · 70k · 100k</h1><p>Điểm tổng và từng thành phần · Train tách biệt evaluate · Giữ cả hai đợt 100k</p><small style="color:#cfe1ef">Xuất @@DATE@@ · HTML nhúng SVG, xem offline không cần Internet</small></header><main><div class="cards">@@CARDS@@</div><section><h2>Đọc kết quả trước</h2><p class="notice">@@CONCLUSION@@</p><ul><li>Mỗi điểm thành phần là <b>tổng reward đã ghi qua các bước của một episode</b>, rồi lấy trung bình giữa các episode. Các trọng số reward đã được áp dụng trong log; không nhân lại lần hai.</li><li>Điểm phạt âm càng gần 0 càng ít phạt. Reward tổng là tổng tám thành phần; không phải phần trăm hay độ chính xác.</li><li>100k gộp 6 lượt là tập dẫn xuất từ hai đợt n=3, không phải một nhóm quan sát độc lập hoặc sáu lượt mới. Giữ nguyên lượt FAILED.</li></ul><p><a href="reward_summary.csv">CSV tổng hợp</a> · <a href="reward_episodes.csv">CSV từng episode và thành phần</a> · <a href="reward_components.csv">CSV thống kê từng thành phần</a> · <a href="reward_data.json">Dữ liệu và nguồn</a> · <a href="VALIDATION.json">Kiểm tra đối chiếu</a></p></section><section><h2>1. Reward lúc train: @@WINDOW@@ episode hoàn tất cuối mỗi run</h2><p>Đây là log khi policy đang học và có khám phá. Cửa sổ @@WINDOW@@ lượt không phải toàn bộ quá trình train, không phải cùng một khoảng step giữa các model, cũng không mặc định trùng <code>ep_rew_mean</code> trên terminal. Model có thể dừng giữa episode; episode chưa kết thúc không được coi là một lượt hoàn chỉnh trong cửa sổ này.</p>@@TRAIN@@<div class="plot">@@PLOT0@@</div><p class="warning">Các run train diễn ra ở các thời điểm/phiên bản môi trường khác nhau. Trọng số reward giống nhau không chứng minh toàn bộ động học, nhiễu và code lúc train giống nhau. Không dùng riêng bảng này để kết luận train thêm gây overfitting.</p></section><section><h2>2. Evaluate deterministic trên cùng cấu hình và đường tham chiếu</h2><p>50k và 70k: mỗi nhóm 3 lượt. 100k: 3 lượt đợt đầu và 3 lượt chạy lại; cùng checkpoint được xác minh bằng SHA-256. Seed 1/2/3 chỉ là nhãn Gym reset, không kiểm soát nhiễu Gazebo/AMCL. Số lượt ít nên không diễn giải chênh lệch như kết luận thống kê chắc chắn.</p>@@EVAL@@<div class="plot">@@PLOT1@@</div><div class="plot">@@PLOT2@@</div></section><section><h2>3. Ý nghĩa các thành phần và những điểm dễ nhầm</h2><ul><li><b>progress:</b> thưởng theo tiến độ dọc đường từ định vị; thay đổi mỗi bước bị chặn trong khoảng ±0,25 m trước khi nhân trọng số.</li><li><b>cross_track / heading:</b> phạt bình phương lỗi ngang/hướng, có nhân dt của bước reward.</li><li><b>vibration:</b> phạt (a_x² + a_y² + a_z²) sau xử lý trọng lực, lấy ở bước RL và nhân dt. Không đồng nhất với RMS a_z riêng lẻ ở telemetry 50 Hz trong báo cáo điều khiển.</li><li><b>effort:</b> phạt độ lớn action SAC đã chuẩn hóa; không phải năng lượng điện hay tổng mô-men đo thực.</li><li><b>smoothness:</b> phạt thay đổi action SAC giữa hai bước; khác tỷ lệ chạm giới hạn slew của lệnh mô-men cuối.</li><li><b>time:</b> phạt theo dt được truyền vào hàm reward, không tự thay bằng tổng thời gian quan sát episode trong báo cáo khác.</li><li><b>terminal:</b> thưởng/phạt theo outcome Nav2 mà môi trường dùng, không phải điều kiện task pass AMCL độc lập của báo cáo benchmark. Hai lượt +30 và một lượt −30 cho trung bình +10.</li><li>Lượt dừng sớm vẫn nằm trong tổng hợp và có thể tích lũy ít phạt hơn vì chạy ngắn; không coi ít phạt như bằng chứng tốt hơn mà bỏ qua outcome/tiến độ.</li><li>Công cụ chỉ cộng <code>steps.jsonl</code>. Không cộng reward lặp từ <code>telemetry.jsonl</code> và không dựng dữ liệu thiếu thành 0.</li></ul><details><summary>Trọng số reward đã kiểm tra giống nhau giữa mọi nguồn</summary><pre>@@WEIGHTS@@</pre></details></section><section><h2>4. Tất cả episode được sử dụng</h2><p>CSV chứa mỗi episode nguồn đúng một lần; tập 100k gộp 6 lượt chỉ xuất trong bảng thống kê dẫn xuất.</p><div class="scroll"><table><thead><tr><th>Loại</th><th>Checkpoint/đợt</th><th>STT</th><th>Episode</th><th>Nav2 outcome</th><th>Reward</th><th>Steps</th></tr></thead><tbody>@@ROWS@@</tbody></table></div></section><section><h2>5. Nguồn dữ liệu và đối chiếu</h2><p>Tổng từ reward_terms được đối chiếu với result.json của evaluate (sai số ≤10⁻⁷), và monitor.csv của train (sai số ≤10⁻⁵ do Monitor làm tròn). Số bước và outcome được đối chiếu. Kiểm tra số step thật trong archive model; kiểm tra hash checkpoint/config/controller/Nav2 của evaluate. Mean ± SD dùng độ lệch chuẩn mẫu; không thay SD bằng standard error.</p><details><summary>Xem đường dẫn nguồn, SHA-256 và bằng chứng kiểm tra</summary><pre>@@PROVENANCE@@</pre></details></section></main></body></html>'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path)
    parser.add_argument('--train-window',type=int,default=10)
    args=parser.parse_args()
    if args.train_window<2:
        parser.error('--train-window cần >=2 để tính SD mẫu')
    out=args.output_dir or ROOT/'benchmark_results'/('reward_comparison_'+datetime.now(ZoneInfo('Asia/Bangkok')).strftime('%Y%m%d_%H%M%S'))
    data=collect(args.train_window)
    out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    data.update(created_local=datetime.now(ZoneInfo('Asia/Bangkok')).isoformat(timespec='seconds'),
        analysis_source_sha256=sha(Path(__file__)),reward_formula_source_sha256=sha(ROOT/'rl_training/features.py'))
    physical=[*data['training'],*data['evaluation']]
    summary=[];components=[];episodes=[]
    for g in physical+[data['combined_100k']]:
        summary.append(dict(kind=g['kind'],group=g['label'],nav_successes=g['nav_successes'],**g['total']))
        for key in KEYS:
            components.append(dict(kind=g['kind'],group=g['label'],component=key,description=NAMES[key],**g['components'][key]))
    for g in physical:
        for e in g['episodes']:
            episodes.append(dict(kind=g['kind'],group=g['label'],episode=e['episode'],outcome=e['outcome'],
                reward=e['total'],steps=e['steps'],duration_s=e['duration_s'],**e['terms'],source=e['source'],source_sha256=e['source_sha256']))
    assert len({e['source'] for e in episodes})==len(episodes)
    write_csv(out/'reward_summary.csv',summary);write_csv(out/'reward_components.csv',components);write_csv(out/'reward_episodes.csv',episodes)
    (out/'reward_data.json').write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False))
    render(data,out,figures(data,out))
    validation=dict(state='PASS',checks=data['validation'],common_reward_weights=True,
        matched_evaluation_conditions=True,source_episodes=len(episodes),
        combined_100k_is_derived=True,training_performed=False,evaluation_performed=False,
        caveats=['Training-window episodes are not controlled evaluation','Small evaluation sample; no significance claim','No old failures removed'])
    (out/'VALIDATION.json').write_text(json.dumps(validation,ensure_ascii=False,indent=2))
    print(json.dumps(dict(report=str(out/'reward_comparison.html'),source_episodes=len(episodes),
        summaries=len(summary),source_unchanged='read-only analysis'),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()

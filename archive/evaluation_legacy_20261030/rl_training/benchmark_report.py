"""HTML độc lập: SVG và dữ liệu tương tác nhúng sẵn, không dùng CDN/server."""
from html import escape
import json
import math
from .benchmark_suite import METRICS,moments


def fmt(value,digits=3):
    return 'N/A' if value is None else f'{value:.{digits}f}'


def cell(stat,scale=1):
    if stat['mean'] is None:return '<span class="na">N/A (n=0)</span>'
    value=fmt(stat['mean']*scale)
    if stat['std'] is not None:value+=' ± '+fmt(stat['std']*scale)
    return value+f' <small>n={stat["n"]}</small>'


def svg_bars(summaries,key,title,unit,scale=1):
    width=1000;left=260;right=120;height=65+40*len(summaries);plot_width=width-left-right
    values=[]
    for s in summaries:
        if key=='success_rate_pct':
            value=s[key];lo=s['success_wilson95_low_pct'];hi=s['success_wilson95_high_pct']
        else:
            stat=s['metrics'][key];value=stat['mean']
            lo=max(0,value-stat['std']) if value is not None and stat['std'] is not None else None
            hi=value+stat['std'] if value is not None and stat['std'] is not None else None
        values.append((value,lo,hi))
    maximum=100 if key=='success_rate_pct' else max([(v[2] if v[2] is not None else v[0])*scale for v in values if v[0] is not None]+[1e-9])*1.1
    chunks=[f'<svg role="img" aria-label="{escape(title)}" viewBox="0 0 {width} {height}"><title>{escape(title)}</title>',
            f'<text x="{left}" y="22" font-size="17">{escape(title)} ({escape(unit)})</text>']
    for tick in range(5):
        x=left+plot_width*tick/4
        chunks.append(f'<line x1="{x}" x2="{x}" y1="33" y2="{height-20}" stroke="#ddd"/><text x="{x}" y="{height-3}" text-anchor="middle" font-size="11">{maximum*tick/4:.2g}</text>')
    for i,(s,(value,lo,hi)) in enumerate(zip(summaries,values)):
        y=48+40*i;color=f'hsl({i*137.508%360:.1f},60%,43%)'
        chunks.append(f'<text x="{left-12}" y="{y+5}" text-anchor="end" font-size="13"><title>{escape(s["label"])}</title>{escape(s["label"][:31])}</text>')
        if value is None:
            chunks.append(f'<text x="{left+10}" y="{y+5}" fill="#777">N/A</text>');continue
        x=left+value*scale/maximum*plot_width
        chunks.append(f'<rect x="{left}" y="{y-10}" width="{max(0,x-left)}" height="20" rx="3" fill="{color}"/>')
        if lo is not None and hi is not None:
            a=left+lo*scale/maximum*plot_width;b=left+hi*scale/maximum*plot_width
            chunks.append(f'<path d="M{a},{y} H{b} M{a},{y-5} V{y+5} M{b},{y-5} V{y+5}" stroke="#222" fill="none"/>')
        chunks.append(f'<text x="{left+plot_width+8}" y="{y+5}" font-size="12">{value*scale:.3f}</text>')
    return ''.join(chunks)+'</svg>'


def write_html(path,report,traces):
    summaries=report['summaries'];labels=[s['label'] for s in summaries]
    settings=report['settings']
    cards=[]
    for s in summaries:
        cards.append(f'<article><h3>{escape(s["label"])}</h3><strong>{s["n_success"]}/{s["n_completed"]} đến đích</strong>'
                     f'<p>{fmt(s["success_rate_pct"],1)}% · Wilson 95% [{fmt(s["success_wilson95_low_pct"],1)}, {fmt(s["success_wilson95_high_pct"],1)}]%</p>'
                     f'<small>Thất bại: {s["n_failed"]} · Hết giờ: {s["n_time_limit"]} · Chưa xong: {s["n_incomplete"]} · Log lỗi: {s["n_invalid"]} · Phiên không có mẫu: {s["startup_only_sessions"]}</small></article>')
    charts=''.join(svg_bars(summaries,key,title,unit,scale) for key,title,unit,scale in [
        ('success_rate_pct','Tỷ lệ đến đích — thanh sai số Wilson 95%','%',1),
        ('cross_track_rmse_time_m','Lỗi ngang RMS cân theo thời gian — TB ± SD','cm',100),
        ('completion_time_s','Thời gian hoàn thành, chỉ lượt thành công — TB ± SD','s',1),
        ('acceleration_rms_m_s2','RMS gia tốc đã ghi — TB ± SD','m/s²',1),
        ('total_torque_rms_nm','RMS mô-men lệnh, trung bình hai bánh — TB ± SD','Nm',1)])
    header='<tr><th>Chỉ số (đơn vị gốc)</th>'+''.join('<th>'+escape(label)+'</th>' for label in labels)+'</tr>'
    rows=[]
    for key,unit in METRICS.items():
        rows.append('<tr><td>'+escape(key)+' <small>'+escape(unit)+'</small></td>'+''.join('<td>'+cell(s['metrics'][key])+'</td>' for s in summaries)+'</tr>')
    reward_keys=sorted({k for s in summaries for k in s['reward_components']})
    for key in reward_keys:
        rows.append('<tr><td>Reward component: '+escape(key)+'</td>'+''.join('<td>'+cell(s['reward_components'].get(key,dict(mean=None,std=None,n=0)))+'</td>' for s in summaries)+'</tr>')
    notices=''.join('<li>'+escape(s)+'</li>' for s in report['confounders']+report['notices'])
    episode_rows=[]
    for group in report['groups']:
        for e in group['episodes']:
            m=e['metrics']
            episode_rows.append('<tr>'+''.join('<td>'+escape(str(v))+'</td>' for v in [group['label'],e['episode'],e['outcome'],
                fmt(m['cross_track_rmse_time_m']*100 if m['cross_track_rmse_time_m'] is not None else None),fmt(m['duration_s']),e['reason'],e['source']])+'</tr>')
    windows=[]
    for label in labels:
        for center in settings['obstacle_x']:
            selected=[w for w in report['windows'] if w['group']==label and w['obstacle']==center and w['outcome']!='INCOMPLETE']
            complete=[w for w in selected if w['window_complete'] and w['metrics']]
            stat=moments(w['metrics']['acceleration_rms_m_s2'] for w in complete)
            windows.append('<tr><td>'+escape(label)+f'</td><td>{center:g}</td><td>{sum(w["coordinate_available"] for w in selected)}</td>'
                           f'<td>{sum(w["samples"]>0 for w in selected)}</td><td>{sum(w["center_crossed"] for w in selected)}</td><td>{len(complete)}</td><td>'+cell(stat)+'</td></tr>')
    window_html=('<p>Không yêu cầu cửa sổ chướng ngại vật.</p>' if not windows else
        '<p>Trục đã chọn: <b>'+escape(settings['obstacle_axis'])+f'</b>; bán kính {settings["window_radius_m"]}m. Đây là vùng tọa độ, không phải sự kiện tiếp xúc/vượt gờ vật lý. Chỉ các cửa sổ đủ hai phía được gộp trong cột RMS.</p>'
        '<div class="scroll"><table><tr><th>Nhóm</th><th>Tâm</th><th>Có tọa độ</th><th>Vào vùng</th><th>Đi qua tâm</th><th>Đủ cửa sổ và mẫu</th><th>Gia tốc RMS (m/s²)</th></tr>'+''.join(windows)+'</table></div>')
    audit_rows=''.join('<tr><td>'+escape(str(a.get('group','')))+'</td><td>'+escape(a['kind'])+'</td><td>'+escape(a.get('source',''))+'</td><td><pre>'+escape(json.dumps(a.get('detail',''),ensure_ascii=False))+'</pre></td></tr>' for a in report['audit'])
    sources=[]
    for g in report['groups']:
        sources.append('<h3>'+escape(g['label'])+'</h3><pre>'+escape(json.dumps(dict(paths=g['paths'],contexts=g['contexts']),ensure_ascii=False,indent=2))+'</pre>')
    payload=json.dumps(dict(traces=traces,settings=settings),ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    method='''<p>Đơn vị thống kê là episode, mỗi episode có trọng số bằng nhau trong trung bình nhóm. SD là độ lệch chuẩn mẫu (n−1); một episode không đủ để ước lượng SD. Wilson 95% dùng cho tỷ lệ đến đích, không phải chứng minh ý nghĩa thống kê giữa model.</p>
<p>Lỗi ngang chính: √(Σdt·e²/Σdt); IAE=Σdt·|e|; ISE=Σdt·e²; ITAE=Σdt·t·|e|. CSV còn chứa RMSE theo số mẫu để đối chiếu công cụ cũ. dt lấy từ step_duration nếu có, nếu không dùng chênh timestamp; timestamp trùng chỉ được chấp nhận ở mẫu kết thúc và có trọng số 0. Không nội suy dữ liệu thiếu. Mỗi chỉ số dùng các mẫu hữu hạn sẵn có; coverage nằm trong JSON/episode CSV.</p>
<p>Gia tốc RMS dùng độ lớn vector; torque RMS dùng trung bình bình phương hai bánh. Jerk và tốc độ thay đổi mô-men là sai phân giữa các mẫu đã ghi, không phải toàn bộ vòng control. Cờ saturation/slew chỉ được thống kê nếu log có cờ; không suy ngược từ torque thưa. Tích phân bình phương mô-men có đơn vị Nm²s, không phải năng lượng tiêu thụ.</p>
<p>Log native steps.jsonl hiện tại có lỗi ngang từ AMCL và gia tốc đã trừ trọng lực. Nó chưa lưu X/Y, ground truth, gyro, vận tốc/target bánh hoặc cờ limiter. CSV ngoài cần tuân theo schema trong hướng dẫn; gia tốc nhập phải là linear acceleration đã xử lý trọng lực. Không tự gỡ trọng lực lần nữa. Vận tốc bánh, sai số AMCL–GT chỉ được tính khi có đủ các cột cần thiết.</p>
<p>Native simulation_time là thời gian từ đầu episode, do đó duration là timestamp cuối. Với time_s/timestamp CSV không có mốc bắt đầu episode, duration là khoảng timestamp cuối−đầu; không suy ra thời gian trước mẫu đầu. Các metric tích phân dùng tổng dt thực ghi. TIME_LIMIT/FAILED vẫn ở mẫu đánh giá; thời gian hoàn thành chỉ dùng SUCCEEDED. INCOMPLETE và log hỏng không bị biến thành thất bại điều khiển.</p>'''
    html=TEMPLATE.replace('@@CARDS@@',''.join(cards)).replace('@@CHARTS@@',charts).replace('@@NOTICES@@',notices)
    html=html.replace('@@TABLE@@',header+''.join(rows)).replace('@@EPISODES@@',''.join(episode_rows)).replace('@@WINDOWS@@',window_html)
    html=html.replace('@@AUDIT@@',audit_rows).replace('@@SOURCES@@',''.join(sources)).replace('@@METHOD@@',method).replace('@@DATA@@',payload)
    path.write_text(html,encoding='utf-8')


TEMPLATE='''<!doctype html><html lang="vi"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Multi-model benchmark</title>
<style>body{font:15px system-ui;background:#eef2f7;color:#183047;margin:0}main{max-width:1300px;margin:auto;padding:28px}header{background:#16334f;color:white;padding:28px;border-radius:14px}section,article{background:white;padding:22px;border-radius:12px;margin:18px 0}h1{margin:8px 0}h2{font-size:22px}.cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:15px}.cards article{margin:0}strong{font-size:22px}p,li{line-height:1.65}.scroll{overflow:auto;max-height:650px}table{width:100%;border-collapse:collapse;white-space:nowrap}td,th{padding:10px;border-bottom:1px solid #ddd;text-align:left}th{position:sticky;top:0;background:#f3f6fa}small,.na{color:#6d7785}pre{white-space:pre-wrap;overflow-wrap:anywhere;max-width:900px}svg{width:100%;font-family:system-ui}select,button{padding:8px;margin:5px}#trace{min-height:280px}.chart{overflow:auto}a{color:#126197}.warn{border-left:5px solid #c58b2a}.trace-label{font-size:12px;fill:#444}@media print{body{background:white}main{padding:0}.scroll{max-height:none}select,button{display:none}section{break-inside:avoid}}</style></head>
<body><main><header><small>OFFLINE · NHIỀU MODEL / NHIỀU RUN · DỮ LIỆU ĐÃ GHI</small><h1>Benchmark và đối sánh thí nghiệm</h1><p>Không chạy robot, không huấn luyện model. Các nhãn do người dùng cung cấp; N/A nghĩa là thiếu dữ liệu.</p><button onclick="window.print()">In / lưu PDF</button></header>
<div class="cards">@@CARDS@@</div>
<section class="warn"><h2>Điều kiện và giới hạn so sánh</h2><ul>@@NOTICES@@</ul></section>
<section><h2>Tổng quan</h2><div class="chart">@@CHARTS@@</div></section>
<section><h2>Tra cứu từng quỹ đạo</h2><label>Nhóm <select id="group"></select></label><label>Episode <select id="episode"></select></label><label>Tín hiệu <select id="metric"></select></label><label>Trục ngang <select id="axis"></select></label><p id="traceNote"></p><svg id="trace" viewBox="0 0 1000 340" role="img" aria-label="Quỹ đạo từng episode"></svg></section>
<section><h2>Chỉ số nhóm: trung bình ± SD và số episode có dữ liệu</h2><div class="scroll"><table>@@TABLE@@</table></div></section>
<section><h2>Cửa sổ chướng ngại vật</h2>@@WINDOWS@@</section>
<section><h2>Mọi episode đã đọc, gồm cả thất bại và chưa hoàn tất</h2><div class="scroll"><table><tr><th>Nhóm</th><th>Episode</th><th>Kết quả</th><th>Lateral RMS (cm)</th><th>Duration (s)</th><th>Lý do</th><th>Nguồn</th></tr>@@EPISODES@@</table></div></section>
<section><h2>Nhật ký chất lượng dữ liệu và lỗi hạ tầng</h2><div class="scroll"><table><tr><th>Nhóm</th><th>Loại</th><th>Nguồn</th><th>Chi tiết</th></tr>@@AUDIT@@</table></div></section>
<section><h2>Phương pháp</h2>@@METHOD@@</section><section><h2>Nguồn và metadata</h2><details><summary>Xem đường dẫn, cấu hình và checkpoint nếu còn trên máy</summary>@@SOURCES@@</details></section>
<section><h2>Xuất dữ liệu</h2><p><a href="comparison_summary.csv">CSV tổng hợp</a> · <a href="comparison_table.tex">Bảng LaTeX</a> · <a href="episode_metrics.csv">CSV episode</a> · <a href="obstacle_windows.csv">CSV cửa sổ gờ</a> · <a href="benchmark_data.json">Dữ liệu và phương pháp JSON</a> · <a href="audit.json">Audit JSON</a></p><p>Biểu đồ và dữ liệu tương tác nằm ngay trong HTML. Có thể chép riêng HTML để xem offline; cần giữ các tệp bên cạnh nếu muốn mở liên kết tải dữ liệu.</p></section>
</main><script id="data" type="application/json">@@DATA@@</script><script>
'use strict';
const payload=JSON.parse(document.getElementById('data').textContent), traces=payload.traces;
const group=document.getElementById('group'), episode=document.getElementById('episode'), metric=document.getElementById('metric'), axis=document.getElementById('axis');
const metricNames={cross_track:'Lỗi ngang (m)',heading_error:'Lỗi hướng (rad)',acceleration_norm:'Độ lớn gia tốc (m/s²)',total_torque_norm:'Độ lớn vector mô-men tổng (Nm)',residual_applied_norm:'Độ lớn residual (Nm)',gyro_norm:'Độ lớn angular velocity (rad/s)',roll:'Roll (rad)',pitch:'Pitch (rad)'};
const axisNames={time:'Thời gian ghi (s)',progress:'Tiến triển dọc đường (m)',amcl_x:'AMCL X (m)',gt_x:'Ground-truth X (m)'};
function options(el,items){el.replaceChildren();for(const [key,text] of items){let o=document.createElement('option');o.value=key;o.textContent=text;el.appendChild(o);}}
options(group,[...new Set(traces.map(t=>t.group))].map(x=>[x,x]));options(metric,Object.entries(metricNames));options(axis,Object.entries(axisNames));
const NS='http://www.w3.org/2000/svg';function elem(tag,attrs,text){let e=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,String(v));if(text!==undefined)e.textContent=text;return e;}
function refreshEpisodes(){options(episode,traces.map((t,i)=>[i,t]).filter(([i,t])=>t.group===group.value).map(([i,t])=>[i,t.episode+' · '+t.outcome]));draw();}
function draw(){const svg=document.getElementById('trace');svg.replaceChildren();let trace=traces[Number(episode.value)];if(!trace){document.getElementById('traceNote').textContent='Không có dữ liệu quỹ đạo.';return;}
let xs=trace.data[axis.value],ys=trace.data[metric.value],pairs=xs.map((x,i)=>[x,ys[i]]).filter(([x,y])=>Number.isFinite(x)&&Number.isFinite(y));
if(!pairs.length){document.getElementById('traceNote').textContent='N/A: log không ghi đủ tín hiệu/trục này; không tự khôi phục cảm biến.';return;}
let xmin=Infinity,xmax=-Infinity,ymin=Infinity,ymax=-Infinity;for(const [x,y] of pairs){xmin=Math.min(xmin,x);xmax=Math.max(xmax,x);ymin=Math.min(ymin,y);ymax=Math.max(ymax,y);}
if(metric.value==='cross_track'){ymin=Math.min(ymin,-payload.settings.max_deviation_m);ymax=Math.max(ymax,payload.settings.max_deviation_m);}if(xmax===xmin)xmax=xmin+1;if(ymax===ymin){ymin-=.1;ymax+=.1;}
let pad=(ymax-ymin)*.08;ymin-=pad;ymax+=pad;const X=x=>70+(x-xmin)/(xmax-xmin)*890,Y=y=>290-(y-ymin)/(ymax-ymin)*255;
if(axis.value===payload.settings.obstacle_axis){for(let center of payload.settings.obstacle_x){let a=Math.max(xmin,center-payload.settings.window_radius_m),b=Math.min(xmax,center+payload.settings.window_radius_m);if(b>a)svg.appendChild(elem('rect',{x:X(a),y:35,width:X(b)-X(a),height:255,fill:'#efcc76',opacity:.25}));}}
for(let i=0;i<=4;i++){let x=xmin+(xmax-xmin)*i/4,y=ymin+(ymax-ymin)*i/4;svg.appendChild(elem('line',{x1:70,y1:Y(y),x2:960,y2:Y(y),stroke:'#ddd'}));svg.appendChild(elem('text',{x:60,y:Y(y)+4,'text-anchor':'end',class:'trace-label'},y.toFixed(3)));svg.appendChild(elem('text',{x:X(x),y:310,'text-anchor':'middle',class:'trace-label'},x.toFixed(2)));}
if(metric.value==='cross_track'){for(let y of [-payload.settings.max_deviation_m,payload.settings.max_deviation_m])svg.appendChild(elem('line',{x1:70,x2:960,y1:Y(y),y2:Y(y),stroke:'#b74335','stroke-dasharray':'5 4'}));}
let points=xs.map((x,i)=>Number.isFinite(x)&&Number.isFinite(ys[i])?[X(x),Y(ys[i])]:null);let d='',pen=false;for(let p of points){if(!p){pen=false;continue;}d+=(pen?' L':' M')+p[0]+','+p[1];pen=true;}svg.appendChild(elem('path',{d,fill:'none',stroke:'#176bba','stroke-width':1.5}));svg.appendChild(elem('text',{x:70,y:20,class:'trace-label'},metricNames[metric.value]));svg.appendChild(elem('text',{x:515,y:335,'text-anchor':'middle',class:'trace-label'},axisNames[axis.value]));document.getElementById('traceNote').textContent=trace.group+' · '+trace.episode+' · '+trace.outcome+' · '+pairs.length+' mẫu; cửa sổ chỉ hiện khi chọn đúng trục đã phân tích.';}
group.addEventListener('change',refreshEpisodes);for(let e of [episode,metric,axis])e.addEventListener('change',draw);refreshEpisodes();
</script></body></html>'''

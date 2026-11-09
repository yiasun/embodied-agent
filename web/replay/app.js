/* Static recorded evidence only. No fetch, ROS bridge, commands or live sockets. */
(() => {
  'use strict';
  const data = window.REPLAY_DATA;
  if (!data || data.schema !== 'offline-replay-v1') throw new Error('Unsupported replay dataset');
  const $ = id => document.getElementById(id);
  const fmt = (n, digits=3) => typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '不适用';
  const NS='http://www.w3.org/2000/svg';
  const el=(tag,attrs={},text=null)=>{const n=document.createElementNS(NS,tag);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));if(text!==null)n.textContent=text;return n;};
  const html=(tag,cls,text)=>{const n=document.createElement(tag);if(cls)n.className=cls;if(text!==undefined)n.textContent=text;return n;};
  const colors={D1:'#087e8b',D2:'#c75d2c',D3:'#c75d2c'};
  let name='D3', experiment=data.experiments.D3, index=0, zoom=false, playing=false, lastWall=null, cursor=0;
  let timeline=[], charts=[];
  const baseline=data.experiments.D1.frames;
  function frameAt(rows,t){let low=0,high=rows.length-1;while(low<high){const mid=Math.ceil((low+high)/2);if(rows[mid].t<=t)low=mid;else high=mid-1;}return low;}
  function path(rows,x,y){let result='';rows.forEach((r,i)=>{const a=x(r),b=y(r);if(a===null||b===null||!Number.isFinite(a)||!Number.isFinite(b))return;const previous=rows[i-1];const connected=r.connect_previous&&previous&&x(previous)!==null&&y(previous)!==null;result+=(connected?'L':'M')+a.toFixed(4)+','+b.toFixed(4);});return result;}
  function mapPath(rows,color,width,opacity=1){return el('path',{d:path(rows,r=>r.robot[0],r=>-r.robot[1]),fill:'none',stroke:color,'stroke-width':width,'stroke-linecap':'round','stroke-linejoin':'round',opacity});}
  function drawMap(){
    const svg=$('map');svg.replaceChildren();svg.setAttribute('viewBox',zoom?'14.7 -27.5 3.4 2.6':'9.5 -32.5 17 13.5');
    data.geometry.boxes.forEach(b=>{const [x,y,x2,y2]=b.bounds;svg.append(el('rect',{x,y:-y2,width:x2-x,height:y2-y,fill:'#cbd6d0'}));});
    if(!zoom)Object.entries(data.geometry.waypoints).forEach(([label,p],i)=>{svg.append(el('circle',{cx:p.x,cy:-p.y,r:.1,fill:'#536c70'}));svg.append(el('text',{x:p.x+.2,y:-p.y-.2,'font-size':.25,fill:'#536c70'},`${i+1} ${label}`));});
    svg.append(mapPath(baseline,colors.D1,zoom?.025:.05,.7));
    svg.append(mapPath(experiment.frames,colors[name],zoom?.022:.04,.2));
    const current=timeline[index], past=experiment.frames.filter(r=>r.t<=current.t);
    svg.append(mapPath(past,colors[name],zoom?.035:.065));
    const obstacle=experiment.frames.filter(r=>r.obstacle).map(r=>({...r,robot:r.obstacle.position}));
    if(obstacle.length)svg.append(mapPath(obstacle,'#6c5b9e',zoom?.02:.04,.4));
    if(current.obstacle){const b=current.obstacle,x=b.position[0],y=-b.position[1];svg.append(el('rect',{x:x-b.size[0]/2,y:y-b.size[1]/2,width:b.size[0],height:b.size[1],fill:'#6c5b9e',stroke:'#fff','stroke-width':.015,transform:`rotate(${-b.yaw*180/Math.PI} ${x} ${y})`}));}
    if(current.robot){
      const [x,y,yaw]=current.robot,p=current.projection;
      if(name!=='D1'){svg.append(el('line',{x1:x,y1:-y,x2:p[0],y2:-p[1],stroke:'#61747a','stroke-width':zoom?.015:.03,'stroke-dasharray':'.06 .05'}));svg.append(el('circle',{cx:p[0],cy:-p[1],r:.08,fill:'#fff',stroke:'#61747a','stroke-width':.02}));}
      svg.append(el('circle',{cx:x,cy:-y,r:data.robot_radius_m,fill:colors[name],opacity:.2}));
      svg.append(el('circle',{cx:x,cy:-y,r:.10,fill:colors[name],stroke:'#fff','stroke-width':.025}));
      svg.append(el('line',{x1:x,y1:-y,x2:x+.27*Math.cos(yaw),y2:-y-.27*Math.sin(yaw),stroke:colors[name],'stroke-width':.04}));
    }else svg.append(el('text',{x:zoom?14.9:10.5,y:zoom?-27.3:-31.6,'font-size':zoom?.09:.28,fill:'#a43c45'},'停止事件时机器人位姿未观测 · 不外推'));
  }
  function renderSummary(){
    $('summary').replaceChildren();
    const stats=[['最终导航结果','4 / 4','SUCCEEDED','最终V4报告；不表示全程有位姿观测'],['第3站观测时长',fmt(experiment.task3.observed_duration_sim_s),'s · sim',`${experiment.task3.sample_count} 条采样 / ${experiment.quality.status}`],['最小采样几何净距',fmt(experiment.task3.minimum_sampled_clearance_m,5),'m',experiment.motion?'圆—矩形计算；非接触测量':'无注入障碍；不适用'],['障碍方向反转',experiment.motion?String(experiment.task3.observed_y_reversals):'—','次',experiment.motion?`${experiment.motion.pattern} / 配置速度 ${fmt(experiment.motion.speed,2)}m/s`:'基线：未注入障碍']];
    stats.forEach(([label,value,unit,detail])=>{const card=html('div','stat');card.append(html('span','label',label));const strong=html('strong','',value+' ');strong.append(html('small','',unit));card.append(strong,html('span','detail',detail));$('summary').append(card);});
    $('verdict').textContent=name==='D3'?'运动实验 FAILED：ROBOT_CLEARANCE_GUARD 提前停止，未完成25秒。原生零速度锁存 / 清理 CLEAN；后续V4四站全部成功。碰撞接触 NOT_MEASURED。':name==='D2'?'单向轨迹到界结束，租约 COMPLETED / CLEAN；最小净距5.518m，未形成近距交会，不能据此声称已验证近距离横穿避障。':'无障碍基线最终四站成功；观测质量 INCOMPLETE，全程导航覆盖 NOT_ASSERTED，不将观测轨迹称为完整导航轨迹。';
    $('tasks').replaceChildren();experiment.tasks.forEach(t=>{const n=html('div','task',`0${t.id} / NAVIGATE`);n.append(html('b','',t.location),html('span','success','● '+t.status),html('div','duration',`${fmt(t.duration_wall_s,2)} s 墙钟 · V4 ${t.recovery}`));$('tasks').append(n);});
    $('replay-name').textContent=`${name} ${experiment.title}`;
    const table=$('comparison-table');table.replaceChildren();const head=html('tr');['指标','D1 无障碍','D2 单向','D3 往返'].forEach(t=>head.append(html('th','',t)));table.append(head);
    const getters=[['最终导航',e=>e.tasks.filter(t=>t.status==='SUCCEEDED').length+'/4 SUCCEEDED'],['运动实验状态',e=>e.experiment_status],['采样窗口 / sim s',e=>fmt(e.task3.observed_duration_sim_s)],['质量 / 全程覆盖',e=>e.quality.status+' / '+e.quality.full_navigation_coverage],['最小采样净距 / m',e=>fmt(e.task3.minimum_sampled_clearance_m,5)],['第3站任务 / wall s',e=>fmt(e.tasks[2].duration_wall_s,2)],['Path变化代理 / 正式重规划',e=>e.path_changes_proxy+' / UNKNOWN'],['接触测量',e=>e.contact_status],['障碍停止 / 清理',e=>(e.motion_stop||'不适用')+' / '+(e.cleanup||'不适用')]];
    getters.forEach(([label,fn])=>{const row=html('tr');row.append(html('td','',label));['D1','D2','D3'].forEach(k=>row.append(html('td','',fn(data.experiments[k]))));table.append(row);});
    $('identity').replaceChildren();Object.entries(experiment.binding).forEach(([key,value])=>{$('identity').append(html('dt','',key),html('dd','',value||'不适用'));});
  }
  function chart(id,series,xkey,ykey,xlabel,ymax,threshold=null){
    const svg=$(id);svg.replaceChildren();const W=660,H=235,L=52,R=20,T=20,B=42;
    const values=series.flatMap(s=>s.rows.filter(r=>typeof r[xkey]==='number'&&typeof r[ykey]==='number'));
    const xmax=Math.max(1,...values.map(r=>r[xkey]));ymax=ymax||Math.max(1,...values.map(r=>r[ykey]))*1.08;
    const x=n=>L+n/xmax*(W-L-R), y=n=>H-B-n/ymax*(H-B-T);
    for(let i=0;i<=4;i++){const ty=y(ymax*i/4),tx=x(xmax*i/4);svg.append(el('line',{x1:L,y1:ty,x2:W-R,y2:ty,stroke:'#e4ebe6'}),el('text',{x:L-8,y:ty+4,'text-anchor':'end'},fmt(ymax*i/4,2)),el('text',{x:tx,y:H-22,'text-anchor':'middle'},fmt(xmax*i/4,1)));}
    svg.append(el('text',{x:(L+W-R)/2,y:H-3,'text-anchor':'middle'},xlabel));
    if(values.length===0){svg.append(el('text',{x:W/2,y:105,'text-anchor':'middle'},'不适用：无注入障碍净距'));return null;}
    if(threshold!==null)svg.append(el('line',{x1:L,y1:y(threshold),x2:W-R,y2:y(threshold),stroke:'#a43c45','stroke-dasharray':'5 4'}));
    series.forEach(s=>{svg.append(el('path',{d:path(s.rows,r=>typeof r[xkey]==='number'&&typeof r[ykey]==='number'?x(r[xkey]):null,r=>typeof r[ykey]==='number'?y(r[ykey]):null),fill:'none',stroke:s.color,'stroke-width':1.8}));});
    const cursorLine=el('line',{x1:L,y1:T,x2:L,y2:H-B,stroke:'#405d65','stroke-dasharray':'3 4'}), dot=el('circle',{r:4,fill:colors[name],stroke:'#fff','stroke-width':1.5});svg.append(cursorLine,dot);
    return {cursorLine,dot,x,y,xkey,ykey};
  }
  function renderCharts(){
    const series=[{rows:baseline,color:colors.D1},...(name==='D1'?[]:[{rows:experiment.frames,color:colors[name]}])];
    charts=[chart('speed-chart',series,'s_ref_m','speed_m_s','D1参考空间弧长 / m',.6),chart('deviation-chart',[{rows:experiment.frames,color:colors[name]}],'s_ref_m','deviation_m','D1参考空间弧长 / m',Math.max(.45,experiment.task3.maximum_sampled_deviation_m*1.1)),chart('clearance-chart',[{rows:experiment.frames,color:colors[name]}],'t','clearance_m','本次观测相对仿真时间 / s',null,.35)].filter(Boolean);
  }
  function renderEvents(){
    const root=$('events');root.replaceChildren();
    if(!experiment.events.length){root.append(html('div','card','本次采集没有原生障碍停止事件；不代表已经测量零碰撞。'));return;}
    experiment.events.forEach(event=>{const card=html('article','event-card'), time=html('div','event-time',fmt(event.t)+' s');time.append(html('small','',`${name}相对仿真时间`));const copy=html('div','event-copy');copy.append(html('h3','',event.reason),html('p','',`原生停止：${event.motion_stop} · 障碍清理：${event.cleanup}`),html('p','',`最后机器人采样 ${fmt(event.last_robot_sample_t)}s，原生事件 ${fmt(event.t)}s；事件时机器人位姿未观测，不推算净距。`));const jump=html('button','event-jump','跳转到原生停止事件 →');jump.onclick=()=>{pause();seek(event.t);$('replay').scrollIntoView({behavior:'smooth'});};copy.append(jump);card.append(time,copy);root.append(card);});
  }
  function update(){
    const row=timeline[index];$('clock').textContent=fmt(row.t)+' s';$('timeline').value=cursor;
    $('sample-state').textContent=row.kind==='sample'?`RECORDED · 源数据行 ${row.source_row}`:'NATIVE STOP · ROBOT NOT OBSERVED';
    $('telemetry').replaceChildren();
    const entries=[['窗口时刻',fmt(row.t)+' sim s'],['位置',row.robot?`${fmt(row.robot[0],2)}, ${fmt(row.robot[1],2)} m`:'未观测'],['差分速度',row.speed_m_s===undefined?'未观测':fmt(row.speed_m_s)+' m/s'],['计算净距',row.clearance_m===undefined?'未观测':fmt(row.clearance_m,5)+' m'],['源任务状态',row.kind==='sample'?'第3站 RUNNING':'机器人状态未同步采样']];
    entries.forEach(([label,value])=>{const n=html('span','',label);n.append(html('b','',value));$('telemetry').append(n);});
    charts.forEach(c=>{const a=row[c.xkey],b=row[c.ykey],valid=typeof a==='number'&&typeof b==='number';c.cursorLine.setAttribute('visibility',valid?'visible':'hidden');c.dot.setAttribute('visibility',valid?'visible':'hidden');if(valid){c.cursorLine.setAttribute('x1',c.x(a));c.cursorLine.setAttribute('x2',c.x(a));c.dot.setAttribute('cx',c.x(a));c.dot.setAttribute('cy',c.y(b));}});
    drawMap();
  }
  function seek(t){cursor=Math.max(0,Math.min(t,timeline.at(-1).t));index=frameAt(timeline,cursor);update();}
  function pause(){playing=false;lastWall=null;$('play').textContent='播放';$('play').setAttribute('aria-label','播放历史轨迹');}
  function choose(key){pause();name=key;experiment=data.experiments[name];timeline=[...experiment.frames,...experiment.events.filter(e=>e.t>=experiment.frames.at(-1).t)].sort((a,b)=>a.t-b.t);$('timeline').max=timeline.at(-1).t;document.querySelectorAll('[data-experiment]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.experiment===name)));renderSummary();renderCharts();renderEvents();seek(0);}
  document.querySelectorAll('[data-experiment]').forEach(b=>b.onclick=()=>choose(b.dataset.experiment));
  $('timeline').oninput=event=>{pause();seek(Number(event.target.value));};
  $('zoom').onclick=()=>{zoom=!zoom;$('zoom').setAttribute('aria-pressed',String(zoom));$('zoom').textContent=zoom?'完整场景':'局部放大';drawMap();};
  $('play').onclick=()=>{if(playing)return pause();if(cursor>=timeline.at(-1).t)seek(0);playing=true;lastWall=null;$('play').textContent='暂停';$('play').setAttribute('aria-label','暂停历史轨迹');};
  function animate(now){if(playing){if(lastWall!==null){seek(cursor+(now-lastWall)/1000*Number($('rate').value));if(cursor>=timeline.at(-1).t)pause();}lastWall=now;}requestAnimationFrame(animate);}
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
  data.limitations.forEach(text=>$('limitations').append(html('li','',text)));
  data.sources.forEach(s=>{const n=html('div','source-row',`${s.experiment} · ${s.role} · ${s.file}`);n.append(html('code','',s.sha256));$('sources').append(n);});
  choose('D3');requestAnimationFrame(animate);
  // Pure selection functions exposed only for offline browser regression checks.
  window.ReplayMath=Object.freeze({frameAt,path});
})();

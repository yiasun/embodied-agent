"""Publication figures from the same sanitized dataset used in the browser."""
from pathlib import Path


def render(data, output):
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib import pyplot as plt
    from matplotlib.font_manager import FontProperties
    from matplotlib.patches import Rectangle
    font_path = Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf')
    if not font_path.is_file():
        raise RuntimeError('Chinese font unavailable; supply a local CJK font before rendering')
    font = FontProperties(fname=font_path)
    plt.rcParams.update({'font.family': ['DejaVu Sans', font.get_name()], 'axes.unicode_minus': False, 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False, 'figure.facecolor': '#fafbf8',
                         'axes.facecolor': '#fafbf8'})
    output.mkdir(parents=True, exist_ok=True)
    d1 = data['experiments']['D1']['frames']; d3 = data['experiments']['D3']['frames']
    colors = ['#087e8b', '#cf612d']
    def line(ax, rows, x, y, **kwargs):
        xs, ys = [], []
        for row in rows:
            if not row['connect_previous'] or row.get(x) is None or row.get(y) is None:
                xs.append(float('nan')); ys.append(float('nan'))
            xs.append(row.get(x, float('nan'))); ys.append(row.get(y, float('nan')))
        ax.plot(xs, ys, **kwargs)
    def finish(fig, name):
        fig.tight_layout(); fig.savefig(output / name, dpi=180); plt.close(fig)
    def map_ax(ax, zoom=False):
        for box in data['geometry']['boxes']:
            x,y,x2,y2 = box['bounds']; ax.add_patch(Rectangle((x,y),x2-x,y2-y,color='#d1d8dc'))
        for rows,color,label in [(d1,colors[0],'D1 第3站已观测轨迹'),(d3,colors[1],'D3 第3站观测窗口')]:
            plotrows = [{**r,'x':r['robot'][0],'y':r['robot'][1]} for r in rows]
            line(ax,plotrows,'x','y',color=color,label=label,linewidth=2)
        obstacle = [r['obstacle']['position'] for r in d3]
        ax.plot([r[0] for r in obstacle],[r[1] for r in obstacle],':',color='#5b4c9d',label='障碍实际采样轨迹')
        if zoom:
            last=d3[-1]; b=last['obstacle']; ax.add_patch(Rectangle((b['position'][0]-.2,b['position'][1]-.2),.4,.4,color='#5b4c9d',alpha=.5))
            ax.scatter(*last['robot'][:2],color=colors[1],s=35,label='最后机器人采样')
            ax.plot([last['robot'][0],last['projection'][0]],[last['robot'][1],last['projection'][1]],'--',color='#52646c',label='D1最近空间投影')
            ax.set_xlim(14.7,18.0);ax.set_ylim(24.9,27.5)
        else:
            for label,point in data['geometry']['waypoints'].items():
                ax.scatter(point['x'],point['y'],color='#344e5a',s=20);ax.annotate(label,(point['x'],point['y']),xytext=(4,5),textcoords='offset points')
            ax.set_xlim(9.6,26.4);ax.set_ylim(19.6,32.4)
        ax.set_aspect('equal');ax.set_xlabel('Gazebo world X（米）');ax.set_ylabel('Gazebo world Y（米）');ax.legend(fontsize=8);ax.grid(alpha=.15)
    fig,ax=plt.subplots(figsize=(10,7));map_ax(ax)
    ax.set_title('D1 / D3 第3站空间轨迹对比（独立实验，非同步机器人）')
    finish(fig,'01_trajectory.png')
    fig,ax=plt.subplots(figsize=(9,6));map_ax(ax,True)
    ax.set_title('局部空间偏移：最大采样点至D1有效折线距离 0.38668米')
    finish(fig,'02_deviation.png')
    fig,ax=plt.subplots(figsize=(10,4.5))
    for rows,color,label in [(d1,colors[0],'D1'),(d3,colors[1],'D3')]:
        line(ax,rows,'s_ref_m','speed_m_s',color=color,label=label,linewidth=1.4)
    ax.set_xlabel('沿D1第3站参考折线的最近投影弧长（米；不是同步时间）');ax.set_ylabel('位姿差分速度（米/秒）')
    ax.set_title('第3站空间对齐速度；同X区间16.5–17.0米均速：D1 0.46031 / D3 0.14418')
    ax.legend();ax.grid(alpha=.2);finish(fig,'03_speed.png')
    fig,ax=plt.subplots(figsize=(10,4.5));line(ax,d3,'t','clearance_m',color=colors[1],label='采样位姿计算净距')
    event=data['experiments']['D3']['events'][-1]
    ax.axhline(.35,ls='--',color='#b23c44',label='原生安全阈值 0.35米')
    ax.axvline(event['t'],ls=':',color='#b23c44',label='原生停止事件 20.439秒（此刻机器人位姿未观测）')
    ax.scatter(d3[-1]['t'],d3[-1]['clearance_m'],color=colors[1],s=30)
    ax.set_xlabel('距D3观测起点的仿真时间（秒）');ax.set_ylabel('保守圆与障碍矩形净距（米）')
    ax.set_title('最小采样几何净距 0.35295米；接触 NOT_MEASURED，不证明零碰撞')
    ax.legend(fontsize=8);ax.grid(alpha=.2);finish(fig,'04_clearance.png')

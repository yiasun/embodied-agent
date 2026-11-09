"""Explicit, reproducible task/spatial comparisons of recorded samples."""
import math


def finite_vector(value,length):
    return isinstance(value,list) and len(value)==length and all(type(v) in (int,float) and math.isfinite(v) for v in value)


def interval_valid(a,b):
    dt=b['sim_time']-a['sim_time']
    return (a.get('task_id')==b.get('task_id') and b.get('segment_valid') is True
            and 0<dt<=0.25)


def task_samples(samples,task=3):
    result=[];previous=None
    for index,row in enumerate(samples):
        if not finite_vector(row.get('robot'),3) or type(row.get('sim_time')) not in (int,float) or not math.isfinite(row['sim_time']):
            raise ValueError('Invalid sample geometry/time')
        if previous is not None and row['sim_time']<=previous:raise ValueError('Non-increasing sample time')
        previous=row['sim_time']
        if row.get('frame')!='gazebo_world':raise ValueError('Unsupported sample frame')
        if row.get('task_id')==task:
            result.append({**row,'source_row':index})
    if len(result)<2:raise ValueError('Insufficient task observations')
    return result


def reference_segments(samples):
    result=[];progress=0.0
    for index,(a,b) in enumerate(zip(samples,samples[1:])):
        if not interval_valid(a,b):continue
        start,end=a['robot'][:2],b['robot'][:2];length=math.dist(start,end)
        if length<=1e-10:continue
        result.append({'a':start,'b':end,'s':progress,'length':length,'index':index})
        progress+=length
    if not result:raise ValueError('No valid baseline segments')
    return result


def project(point,segments):
    if not finite_vector(point,2):raise ValueError('Invalid query point')
    best=None
    for segment in segments:
        a,b=segment['a'],segment['b'];dx,dy=b[0]-a[0],b[1]-a[1]
        u=max(0,min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
        q=[a[0]+u*dx,a[1]+u*dy];distance=math.dist(point,q)
        if best is None or distance<best['distance_m']:
            best={'distance_m':distance,'point':q,'s_ref_m':segment['s']+u*segment['length'],
                  'segment_index':segment['index'],'fraction':u}
    if best is None:raise ValueError('Empty reference')
    return best


def clipped_fraction(a,b,low,high):
    if not low<high:raise ValueError('Empty spatial window')
    if abs(b-a)<1e-12:return (0,1) if low<=a<=high else None
    ends=sorted([(low-a)/(b-a),(high-a)/(b-a)])
    start,end=max(0,ends[0]),min(1,ends[1])
    return (start,end) if end>start else None


def spatial_speed(samples,low=16.5,high=17.0):
    duration=distance=0.0;intervals=0
    for a,b in zip(samples,samples[1:]):
        if not interval_valid(a,b):continue
        window=clipped_fraction(a['robot'][0],b['robot'][0],low,high)
        if window is None:continue
        fraction=window[1]-window[0];intervals+=1
        duration+=(b['sim_time']-a['sim_time'])*fraction
        distance+=math.dist(a['robot'][:2],b['robot'][:2])*fraction
    return {'x_window_m':[low,high],'duration_sim_s':duration,'distance_m':distance,
            'mean_speed_m_s':distance/duration if duration>0 else None,'interval_count':intervals,
            'method':'valid intervals clipped at X boundaries; distance / simulation duration; all visits included'}


def summarize(samples):
    length=longest=continuous=0.0
    for a,b in zip(samples,samples[1:]):
        if not interval_valid(a,b):continuous=0;continue
        dt=b['sim_time']-a['sim_time'];distance=math.dist(a['robot'][:2],b['robot'][:2]);length+=distance
        continuous=continuous+dt if distance/dt<0.1 else 0
        longest=max(longest,continuous)
    return {'observed_duration_sim_s':samples[-1]['sim_time']-samples[0]['sim_time'],
            'observed_path_length_m':length,'longest_observed_low_speed_sim_s':longest,'low_speed_threshold_m_s':0.1}


def clearance(robot,radius,box):
    dx,dy=robot[0]-box['position'][0],robot[1]-box['position'][1]
    c,s=math.cos(box['yaw']),math.sin(box['yaw']);x,y=c*dx+s*dy,-s*dx+c*dy
    qx,qy=abs(x)-box['size'][0]/2,abs(y)-box['size'][1]/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-radius


def reversals(samples,epsilon=1e-5):
    signs=[]
    for a,b in zip(samples,samples[1:]):
        if not interval_valid(a,b) or a.get('obstacle') is None or b.get('obstacle') is None:continue
        dy=b['obstacle']['position'][1]-a['obstacle']['position'][1]
        if abs(dy)>epsilon:signs.append(1 if dy>0 else -1)
    return sum(a!=b for a,b in zip(signs,signs[1:]))


def frames(samples,reference,radius=0.22):
    origin=samples[0]['sim_time'];result=[];own_progress=0
    for index,row in enumerate(samples):
        speed=None;valid=index>0 and interval_valid(samples[index-1],row)
        if valid:
            distance=math.dist(samples[index-1]['robot'][:2],row['robot'][:2]);own_progress+=distance
            speed=distance/(row['sim_time']-samples[index-1]['sim_time'])
        box=row.get('obstacle')
        if box is not None:
            if (not finite_vector(box.get('position'),3) or not finite_vector(box.get('size'),3)
                    or any(v<=0 for v in box['size']) or type(box.get('yaw')) not in (int,float) or not math.isfinite(box['yaw'])):
                raise ValueError('Invalid obstacle')
            safe_box={k:box[k] for k in ['position','size','yaw']}
            gap=clearance(row['robot'],radius,safe_box)
            stored=row.get('geometric_clearance_m')
            if stored is not None and abs(stored-gap)>1e-7:raise ValueError('Stored geometry differs from recomputation')
        else:safe_box=None;gap=None
        projection=project(row['robot'][:2],reference)
        result.append({'t':row['sim_time']-origin,'source_row':row['source_row'],
            'robot':row['robot'],'obstacle':safe_box,'speed_m_s':speed,'clearance_m':gap,
            'deviation_m':projection['distance_m'],'projection':projection['point'],
            's_ref_m':projection['s_ref_m'],'s_own_m':own_progress,'connect_previous':valid,
            'task_id':row['task_id'],'kind':'sample'})
    return result

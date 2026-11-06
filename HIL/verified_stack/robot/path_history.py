"""Run-scoped path history and offline comparison; simulation truth is evaluation only."""
import csv
import json
import math
from bisect import bisect_right
from pathlib import Path

import yaml


def read_jsonl(path):
    """Keep complete records if a forcibly stopped writer left a partial last line."""
    records=[]
    if path.exists():
        for line in path.read_text().splitlines():
            try:records.append(json.loads(line))
            except json.JSONDecodeError:continue
    return records


def length(points):
    return sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))


def world_points(record,spawn):
    if record['frame']=='world':return record['xy']
    if record['frame']!='map':return []
    c,s=math.cos(spawn[5]),math.sin(spawn[5])
    return [[spawn[0]+c*x-s*y,spawn[1]+s*x+c*y] for x,y in record['xy']]


class PathRecorder:
    def __init__(self,output):
        self.path=Path(output)/'planned_paths.jsonl';self.path.touch(exist_ok=True)
        self.previous={};self.sequence=0

    def record(self,kind,topic,msg,now):
        xy=[[float(p.pose.position.x),float(p.pose.position.y)] for p in msg.poses]
        if not math.isfinite(now) or any(not math.isfinite(v) for p in xy for v in p):return False
        frame=msg.header.frame_id
        # Header refresh alone is not a new plan; geometry changes and clears are.
        signature=(frame,tuple(map(tuple,xy)))
        if self.previous.get(topic)==signature:return False
        self.previous[topic]=signature;self.sequence+=1
        entry=dict(sequence=self.sequence,sim_time=now,
                   stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                   kind=kind,topic=topic,frame=frame,xy=xy,provenance='ros_topic')
        with self.path.open('a') as stream:stream.write(json.dumps(entry,allow_nan=False)+'\n')
        return True


def export_paths(output):
    """Export world-frame actual samples, all route revisions and one overview plot.

    Planned map coordinates use only the fixed spawn origin/heading from run.yaml.
    No alignment to actual motion is fitted, so localization errors remain visible.
    """
    output=Path(output);metadata=yaml.safe_load((output/'run.yaml').read_text())
    events=read_jsonl(output/'events.jsonl')
    summary=json.loads((output/'summary.json').read_text()) if (output/'summary.json').exists() else {}
    run_status=json.loads((output/'run_status.json').read_text()) if (output/'run_status.json').exists() else {}
    sent=sorted((e['sim_time'],e['target']) for e in events if e.get('event')=='goal_sent')
    times=[t for t,_ in sent]
    def target_at(t):
        i=bisect_right(times,t)-1
        return sent[i][1] if i>=0 else 'startup'
    terminal=[e['sim_time'] for e in events if e.get('event')=='estimated_mission_completed']
    end=min(terminal) if terminal else float('inf')
    truth=sorted((r for r in read_jsonl(output/'truth.jsonl') if r['sim_time']<=end
                  and math.isfinite(r['sim_time']) and len(r.get('xyz',[]))>=3
                  and all(math.isfinite(v) for v in r['xyz'])),key=lambda r:r['sim_time'])
    with (output/'actual_path.csv').open('w',newline='') as stream:
        writer=csv.writer(stream);writer.writerow(['sim_time_s','world_x_m','world_y_m','world_z_m','active_target','source'])
        for r in truth:writer.writerow([r['sim_time'],*r['xyz'][:3],target_at(r['sim_time']),'gazebo_truth'])
    history_path=output/'planned_paths.jsonl'
    if not history_path.exists():
        # Older runs contain only sampled status paths, not every published revision.
        records=[];previous={}
        for r in read_jsonl(output/'status.jsonl'):
            for key,kind in [('path','global'),('local_path','committed')]:
                if key not in r:continue
                xy=r[key]
                if previous.get(kind)==xy:continue
                previous[kind]=xy
                records.append(dict(sequence=len(records)+1,sim_time=r['sim_time'],stamp=None,
                    kind=kind,topic=None,frame='map',xy=xy,provenance='sampled_status_reconstruction'))
        history_path.write_text(''.join(json.dumps(r)+'\n' for r in records))
    plans=[p for p in read_jsonl(history_path) if p['sim_time']<=end]
    sx,sy=metadata['world_spawn'][:2]
    def world_path(p):return world_points(p,metadata['world_spawn'])
    first={}
    for p in plans:
        target=target_at(p['sim_time'])
        if p['kind']=='global' and p['xy'] and world_path(p):first.setdefault(target,p)
    requested=[t['name'] for t in metadata['mission']['targets']]
    actual_length=length([r['xyz'] for r in truth])
    report=dict(route_success=bool(summary.get('route_success',False)),outcome=run_status.get('outcome','unknown'),
        requested_targets=requested,verified_completed_targets=summary.get('verified_completed_targets',[]),
        actual_source='Gazebo truth; evaluation only',actual_frame='world',actual_samples=len(truth),
        actual_distance_xy_m=actual_length,actual_first_time_s=truth[0]['sim_time'] if truth else None,
        actual_last_time_s=truth[-1]['sim_time'] if truth else None,
        planned_updates_by_kind={kind:sum(p['kind']==kind for p in plans) for kind in sorted({p['kind'] for p in plans})},
        planned_provenance=sorted({p.get('provenance','unknown') for p in plans}),
        unsupported_plan_frames=sorted({p['frame'] for p in plans if p['frame'] not in ('map','world')}),
        planned_world_alignment='Fixed world_spawn translation and yaw; no fit to actual trajectory',
        first_published_route_lengths_m={target:length(world_path(p)) for target,p in first.items()},
        notes=['Route revisions are alternatives, not one continuous planned path.',
               'First published routes may be partial; their lengths are not optimal-route benchmarks.',
               'Actual distance is sampled planar travel, including recovery and pre-mission settling.',
               'Interrupted runs end at the last recorded position; no arrival is fabricated.'])
    (output/'path_summary.json').write_text(json.dumps(report,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig,ax=plt.subplots(figsize=(10,8))
    for kind,color,alpha,label in [('global','#6f91c7',.18,'Global route revisions'),('committed','#22a06b',.28,'Committed local sections')]:
        segments=[world_path(p) for p in plans if p['kind']==kind and len(world_path(p))>1]
        if segments:ax.add_collection(LineCollection(segments,colors=color,alpha=alpha,linewidths=1,label=label))
    for i,p in enumerate(first.values()):
        xy=world_path(p);ax.plot([v[0] for v in xy],[v[1] for v in xy],'--',color='#2164b5',lw=1.7,label='First route for each target' if i==0 else None)
    if truth:
        ax.plot([r['xyz'][0] for r in truth],[r['xyz'][1] for r in truth],color='#242424',lw=2,label='Actual traveled path',zorder=4)
        ax.scatter(*truth[-1]['xyz'][:2],marker='x',color='#242424',s=70,zorder=6,label='Actual endpoint')
    ax.scatter(sx,sy,color='#d32f2f',s=65,zorder=6,label='Spawn');ax.annotate('spawn',(sx,sy),xytext=(6,6),textcoords='offset points')
    for i,name in enumerate(requested):
        x,y=metadata['world_targets'][name][:2];last=i==len(requested)-1
        ax.scatter(x,y,color='#ed8b23' if last else '#2164b5',marker='*' if last else 'o',s=130 if last else 55,zorder=5)
        ax.annotate(name,(x,y),xytext=(6,6),textcoords='offset points')
    title=f"{metadata['map']} / {metadata['robot']} / {metadata['model']}"
    state='Verified requested route complete' if report['route_success'] else 'Requested route incomplete / unverified'
    ax.set(title=f'{title}\n{state} · actual travel {actual_length:.1f} m',xlabel='World X (m)',ylabel='World Y (m)')
    ax.autoscale_view();ax.margins(.1);ax.set_aspect('equal',adjustable='datalim');ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
    provenance='Planned history reconstructed from sampled old logs.' if 'sampled_status_reconstruction' in report['planned_provenance'] else 'All received changed paths recorded; repeated identical geometry omitted.'
    if not plans:provenance='No planned path was recorded in this run.'
    fig.text(.5,.015,provenance+' Fixed spawn alignment; actual path is simulator truth.',ha='center',fontsize=8)
    fig.tight_layout(rect=(0,.04,1,1));fig.savefig(output/'path_comparison.png',dpi=160);plt.close(fig)
    return report

"""Offline motion-primitive compiler for a differential/skid-steer robot.

Cubic Hermite/Bezier curves hit exact lattice endpoints and heading bins.
Curvature, speed, acceleration and collision-sampling limits are checked
before the library is written. The runtime planner only loads this library.
"""
import json
import math
from pathlib import Path
import numpy as np

def _curve(start_angle,end_angle,dx,dy,res,samples):
    p0=np.zeros(2)
    p3=np.array((dx*res,dy*res),float)
    chord=float(np.linalg.norm(p3))
    p1=chord/3*np.array((math.cos(start_angle),math.sin(start_angle)))
    p2=p3-chord/3*np.array((math.cos(end_angle),math.sin(end_angle)))
    t=np.linspace(0.,1.,samples)
    u=t[:,None]
    xyz=(1-u)**3*p0+3*(1-u)**2*u*p1+3*(1-u)*u*u*p2+u**3*p3
    d=3*(1-u)**2*(p1-p0)+6*(1-u)*u*(p2-p1)+3*u*u*(p3-p2)
    dd=6*(1-u)*(p2-2*p1+p0)+6*u*(p3-2*p2+p1)
    norm=np.linalg.norm(d,axis=1)
    if np.min(norm)<1e-6:return None
    heading=np.unwrap(np.arctan2(d[:,1],d[:,0]))
    curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/norm**3
    step=np.linalg.norm(np.diff(xyz,axis=0),axis=1)
    arclength=np.r_[0.,np.cumsum(step)]
    return xyz,heading,curvature,arclength

def _profile(curvature,arclength,max_v,max_w,max_a,lateral_a):
    k=np.maximum(np.abs(curvature),1e-6)
    limits=np.minimum(np.minimum(max_v,max_w/k),np.sqrt(lateral_a/k))
    speed=np.array(limits)
    speed[0]=0.;speed[-1]=0.
    for i in range(1,len(speed)):
        ds=arclength[i]-arclength[i-1]
        speed[i]=min(speed[i],math.sqrt(speed[i-1]**2+2*max_a*ds))
    for i in range(len(speed)-2,-1,-1):
        ds=arclength[i+1]-arclength[i]
        speed[i]=min(speed[i],math.sqrt(speed[i+1]**2+2*max_a*ds))
    duration=sum(2*(arclength[i+1]-arclength[i])/max(speed[i]+speed[i+1],1e-6)
                 for i in range(len(speed)-1))
    return speed,duration

def compile_library(robot,cfg,output):
    res=float(cfg['planning_resolution'])
    headings=int(cfg['lattice_headings'])
    max_v=min(float(cfg['lattice_cruise_speed']),float(robot['max_linear_speed']))
    max_w=float(robot['max_angular_speed'])
    max_a=float(robot['linear_acceleration'])
    lateral_a=float(cfg['lattice_lateral_acceleration'])
    sample_step=float(cfg['lattice_sample_spacing'])
    if not (headings>=8 and headings%4==0 and 0<res<=.5 and
            min(max_v,max_w,max_a,lateral_a,sample_step)>0):
        raise ValueError('Invalid lattice/robot limits')
    lengths=tuple(float(v) for v in cfg['lattice_primitive_lengths'])
    if not lengths or min(lengths)<2*res:
        raise ValueError('Primitive lengths must be at least two cells')
    primitives=[]
    max_curvature=min(float(cfg['lattice_max_curvature']),max_w/float(cfg['lattice_min_curve_speed']))
    for h in range(headings):
        start=h*2*math.pi/headings
        for dh in (-2,-1,0,1,2):
            end=start+dh*2*math.pi/headings
            candidates=[]
            for nominal in lengths:
                delta=end-start
                if abs(delta)>1e-9:
                    radius=nominal/delta
                    ideal=np.array((radius*(math.sin(end)-math.sin(start)),
                                    -radius*(math.cos(end)-math.cos(start))))
                else:
                    ideal=nominal*np.array((math.cos(start),math.sin(start)))
                cx,cy=np.rint(ideal/res).astype(int)
                for dx in range(cx-1,cx+2):
                    for dy in range(cy-1,cy+2):
                        chord=math.hypot(dx*res,dy*res)
                        if chord<2*res or chord>max(lengths)+res:continue
                        p=np.array((dx*res,dy*res))
                        if np.dot(p,(math.cos(start),math.sin(start)))<res:continue
                        lateral=np.dot(p,(-math.sin(start),math.cos(start)))
                        if dh and lateral*dh<-.05:continue
                        count=max(21,int(math.ceil(chord/sample_step))*2+1)
                        curve=_curve(start,end,dx,dy,res,count)
                        if curve is None:continue
                        xy,yaw,k,s=curve
                        if np.max(np.abs(k))>max_curvature+1e-9:continue
                        if s[-1]>chord*1.2:continue
                        if np.max(np.linalg.norm(np.diff(xy,axis=0),axis=1))>sample_step+.01:continue
                        speed,duration=_profile(k,s,max_v,max_w,max_a,lateral_a)
                        if np.max(np.abs(k*speed))>max_w+1e-6:continue
                        penalty=np.linalg.norm(p-ideal)+.03*s[-1]+.03*np.max(np.abs(k))
                        candidates.append((penalty,dx,dy,xy,yaw,k,s,speed,duration))
            unique=set()
            # Two lengths per heading transition make narrow/open-space choices possible.
            for _,dx,dy,xy,yaw,k,s,speed,duration in sorted(candidates,key=lambda item:item[0]):
                if (dx,dy) in unique:continue
                unique.add((dx,dy))
                primitives.append(dict(id=len(primitives),kind='curve',start_h=h,end_h=(h+dh)%headings,
                                       dx=dx,dy=dy,length=float(s[-1]),duration=float(duration),
                                       max_curvature=float(np.max(np.abs(k))),
                                       samples=[[float(x),float(y),float(a),float(c),float(v),float(d)]
                                                for (x,y),a,c,v,d in zip(xy,yaw,k,speed,s)]))
                if len(unique)==2:break
            if not unique:raise ValueError(f'No feasible primitive for heading {h}, change {dh}')
        for dh in (-1,1):
            steps=max(5,abs(dh)*5)
            angle=np.linspace(start,start+dh*2*math.pi/headings,steps)
            duration=abs(angle[-1]-angle[0])/max_w
            primitives.append(dict(id=len(primitives),kind='rotate',start_h=h,end_h=(h+dh)%headings,
                                   dx=0,dy=0,length=0.,duration=duration,
                                   max_curvature=0.,
                                   samples=[[0.,0.,float(a),0.,0.,0.] for a in angle]))
    data=dict(version=1,resolution=res,headings=headings,
              limits=dict(max_linear=max_v,max_angular=max_w,linear_acceleration=max_a,
                          lateral_acceleration=lateral_a,max_curvature=max_curvature),
              primitives=primitives)
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(data,separators=(',',':')))
    return data

def load_library(path,robot,cfg):
    data=json.loads(Path(path).read_text())
    if data['version']!=1 or data['headings']!=cfg['lattice_headings'] or not math.isclose(data['resolution'],cfg['planning_resolution']):
        raise ValueError('Motion primitive library does not match lattice settings')
    if data['limits']['max_linear']>robot['max_linear_speed']+1e-9 or data['limits']['max_angular']>robot['max_angular_speed']+1e-9:
        raise ValueError('Motion primitive library exceeds robot limits')
    return data

"""Directed motion graph over precompiled differential-drive primitives."""
import math
import numpy as np
from .graph_search import GraphSearchBase,INF

GOAL=(-1,-1,-1)

class MotionGraph(GraphSearchBase):
    algorithm='motion_graph'
    def __init__(self):
        super().__init__()
        self.goal_cell=None
        self.library=None
        self.outgoing={}
        self.incoming={}
        self.transition={}
        self.reverse_impact={}

    def configure(self,data):
        self.library=data
        self.outgoing={h:[] for h in range(data['headings'])}
        self.incoming={h:[] for h in range(data['headings'])}
        self.transition={}
        self.reverse_impact={}
        res=data['resolution']
        for primitive in data['primitives']:
            h=primitive['start_h'];end=primitive['end_h']
            self.outgoing[h].append(primitive)
            self.incoming[end].append(primitive)
            key=(h,primitive['dx'],primitive['dy'],end)
            if key in self.transition:
                raise ValueError(f'Duplicate lattice transition {key}')
            self.transition[key]=primitive
            if primitive['kind']=='rotate':
                touched={(0,0)}
            else:
                touched={(int(math.floor(p[0]/res+.5)),int(math.floor(p[1]/res+.5)))
                         for p in primitive['samples']}
            for ox,oy in touched:
                for ax in (-1,0,1):
                    for ay in (-1,0,1):
                        self.reverse_impact.setdefault((ox+ax,oy+ay),set()).add((h,primitive['dx'],primitive['dy']))
        if any(not self.outgoing[h] for h in self.outgoing):
            raise ValueError('Motion library has an empty heading')
        return self

    def successors(self,s):
        if s==GOAL:return ()
        x,y,h=s
        states={(x+p['dx'],y+p['dy'],p['end_h']) for p in self.outgoing[h]}
        if (x,y)==self.goal_cell:states.add(GOAL)
        return states

    def predecessors(self,s):
        if s==GOAL:
            x,y=self.goal_cell
            return {(x,y,h) for h in range(self.library['headings'])}
        x,y,h=s
        return {(x-p['dx'],y-p['dy'],p['start_h']) for p in self.incoming[h]}

    def adjacent(self,s):
        return set(self.successors(s))|set(self.predecessors(s))

    def primitive(self,a,b):
        if b==GOAL:return None
        return self.transition.get((a[2],b[0]-a[0],b[1]-a[1],b[2]))

    def world_samples(self,state,primitive,start_xy=None):
        grid=self.grid
        xy=np.asarray(start_xy if start_xy is not None else grid.xy(state[:2]),float)
        offset=np.asarray([[p[0],p[1]] for p in primitive['samples']])
        return xy+offset@grid.rot.T

    @staticmethod
    def cell_weight(grid,cells=None):
        if cells is None:
            distance=grid.distance
            known=grid.known
        else:
            x,y=cells[:,0],cells[:,1]
            distance=grid.distance[y,x]
            known=grid.known[y,x]
        base=np.where(known,1.,grid.cfg['unknown_cost'])
        base=base+grid.cfg['clearance_cost']/np.maximum(grid.res,distance)
        comfort=grid.radius+grid.cfg.get('lattice_comfort_distance_m',0.)
        shortfall=np.maximum(0.,comfort-distance)
        return base+grid.cfg.get('lattice_comfort_weight',0.)*shortfall**2

    def _cost(self,a,b):
        grid=self.grid
        if a==GOAL or not grid.free(a[:2]):return INF
        if b==GOAL:return 0. if a[:2]==self.goal_cell else INF
        if not grid.free(b[:2]):return INF
        primitive=self.primitive(a,b)
        if primitive is None:return INF
        samples=self.world_samples(a,primitive,self.start_xy if a==self.start else None)
        if primitive['kind']=='rotate':
            if not grid.motion_clear(samples,grid.hard_radius):return INF
        elif not grid.motion_clear(samples):
            return INF
        local=(samples-grid.origin)@grid.rot
        cells=np.floor((local-grid.lower)/grid.res).astype(int)
        if ((cells<0).any() or (cells[:,0]>=grid.blocked.shape[1]).any()
                or (cells[:,1]>=grid.blocked.shape[0]).any()):
            return INF
        weight=float(np.mean(self.cell_weight(grid,cells)))
        heading_change=abs(float(primitive['samples'][-1][2]-primitive['samples'][0][2]))
        return (primitive['duration']*weight+
                grid.cfg.get('lattice_turn_cost',0.)*heading_change)

    def update(self,grid,start,goal,start_xy,goal_xy):
        if self.library is None:raise ValueError('Motion library has not been loaded')
        if self.goal_cell!=goal or not self.same_geometry(grid):
            self.goal_cell=goal
            self.reset(grid,start,GOAL,start_xy,goal_xy)
            return 'reset'
        old=self.grid
        old_w=self.cell_weight(old)
        new_w=self.cell_weight(grid)
        changed=np.argwhere((old.blocked!=grid.blocked)|(np.abs(old_w-new_w)>1e-3))
        if len(changed)>grid.blocked.size*grid.cfg['dstar_reset_fraction']:
            self.goal_cell=goal
            self.reset(grid,start,GOAL,start_xy,goal_xy)
            return f'reset_broad_change:{len(changed)}'
        old_start=self.start
        self.grid=grid;self.start=start;self.start_xy=tuple(start_xy);self.goal_xy=tuple(goal_xy)
        self.edge_cache.clear()
        impacted=set()
        for y,x in changed:
            for (ox,oy),headings in self.reverse_impact.items():
                sx=int(x)-ox;sy=int(y)-oy
                if not(0<=sx<grid.blocked.shape[1] and 0<=sy<grid.blocked.shape[0]):continue
                for h,_,_ in headings:
                    candidate=(sx,sy,h)
                    if self.tag.get(candidate)=='CLOSED':impacted.add(candidate)
        if old_start!=start:impacted.update((old_start,start))
        for state in impacted:
            if state==GOAL or self.tag.get(state)=='NEW':continue
            options=[(self.cost(state,n)+self.h.get(n,INF),n)
                     for n in self.successors(state)]
            best,parent=min(options,default=(INF,None))
            current=self.h.get(state,INF)
            if not (math.isfinite(best) and math.isfinite(current)
                    and math.isclose(best,current,rel_tol=1e-8,abs_tol=1e-8)):
                if parent is not None and math.isfinite(best):self.parent[state]=parent
                self.insert(state,best)
        if len(changed):self.repairs+=1
        return f'updated:{len(changed)}'

    def path(self):
        if not self.grid.free(self.goal_cell):return [],'goal_occupied'
        if not self.grid.free(self.start[:2]):return [],'start_occupied'
        if not math.isfinite(self.h.get(self.start,INF)):return [],'no_route'
        states=[self.start];seen={self.start};segments=[]
        for _ in range(self.grid.blocked.size*self.library['headings']):
            a=states[-1]
            if a==GOAL:return segments,self.algorithm
            b=self.parent.get(a)
            if b is None or b in seen or not math.isfinite(self.cost(a,b)):
                return [],'no_route'
            if b==GOAL:return segments,self.algorithm
            primitive=self.primitive(a,b)
            if primitive is None:return [],'invalid_transition'
            segments.append((a,b,primitive))
            states.append(b);seen.add(b)
        return [],'route_cycle'

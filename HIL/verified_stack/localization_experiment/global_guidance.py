"""Optional known-free global guidance. No simulator or ground-truth imports."""
import math
from collections import deque
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree
from exploration_memory.core import safe_space,connected_distances,observe

def path_from_distances(grid,dist,target):
    cell=grid.cell(target)
    if not grid.inside(cell) or not math.isfinite(dist[cell[1],cell[0]]):return []
    route=[cell]
    while dist[cell[1],cell[0]]>0:
        x,y=cell
        neighbors=[(a,b) for a,b in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)) if 0<=b<dist.shape[0] and 0<=a<dist.shape[1] and dist[b,a]<dist[y,x]]
        if not neighbors:return []
        cell=min(neighbors,key=lambda c:dist[c[1],c[0]]);route.append(cell)
    return grid.world(np.array(route[::-1])).tolist()

class Guidance:
    def frontier_extra_cost(self,grid,dist,start,point):
        """Optional isolated scoring experiment; baseline cost is unchanged."""
        return 0.
    def __init__(self,boundary_follow=False):
        from localization_experiment.boundary_guidance import BoundaryPreference
        self.boundary=BoundaryPreference() if boundary_follow else None
        self.visits={};self.target=None;self.goal=None;self.best=float('inf');self.progress_at=0;self.blocked={}
        # Populated only when passive full-state observation is explicitly
        # enabled by the navigator. It never contributes to selection.
        self.observe_full_state=False;self.last_full_state=None
    def choose(self,grid,start,goal,clearance,cfg,now,connector_clear=None,execution_mask=None):
        self.last_full_state=None
        if self.goal!=tuple(goal):
            self.goal=tuple(goal);self.target=None;self.visits={};self.blocked={}
        free,safe,_=safe_space(grid,clearance,25)
        if execution_mask is not None:
            safe &= execution_mask
        source=grid.cell(start);prefix=[]
        if not grid.inside(source):return [],{'reason':'source_outside_map'}
        if not safe[source[1],source[0]]:
            # The scan-map starts at the sensor origin; the full inflated
            # footprint can straddle that observed-map boundary at startup.
            # Connect only across observed FREE centreline cells, with an
            # independent unchanged live swept-footprint clearance check.
            yy,xx=np.nonzero(safe);points=grid.world(np.column_stack([xx,yy]))
            ds=np.linalg.norm(points-np.asarray(start),axis=1)
            options=np.flatnonzero(ds<=2*clearance)
            selected=next((i for i in sorted(options,key=lambda i:ds[i]) if grid.line_in_mask(start,points[i],free) and connector_clear is not None and connector_clear(start,points[i])),None)
            if selected is None:return [],{'reason':'source_not_known_safe'}
            source=(int(xx[selected]),int(yy[selected]));prefix=[list(start)]
        dist=connected_distances(safe,source,grid.resolution)
        if not np.isfinite(dist).any():return [],{'reason':'source_not_known_safe'}
        goal_path=path_from_distances(grid,dist,goal)
        if goal_path:
            self.target=tuple(goal)
            return prefix+goal_path,{'reason':'known_free_goal_route','target':self.target}
        if self.boundary:self.boundary.prepare(grid,start,goal)
        # Re-evaluate frontiers on every current map. Reachability alone is
        # insufficient: an old viewpoint may no longer reveal unknown space.
        # No visited-zone or failed-zone history contributes to this choice.
        previous_target=self.target
        obs=observe(grid,start,clearance,cfg)
        # A single connected boundary may wrap the entire observed map. One
        # centroid viewpoint would discard all other unexplored directions.
        # Sample distinct reachable viewpoints across that boundary instead.
        obstacle_distance=ndimage.distance_transform_edt(grid.data<65)*grid.resolution
        traversable_unknown=(grid.data==-1)&(obstacle_distance>clearance+grid.resolution*math.sqrt(2)/2)
        boundary=free & ndimage.binary_dilation(traversable_unknown)
        by,bx=np.nonzero(boundary);ry,rx=np.nonzero(np.isfinite(dist))
        sampled=[]
        boundary_labels,_=ndimage.label(boundary,structure=np.ones((3,3),bool)) if self.observe_full_state else (None,0)
        if len(bx) and len(rx):
            separation,nearest=cKDTree(np.column_stack([bx,by])).query(np.column_stack([rx,ry]))
            step=max(1,int(round(.75/grid.resolution)))
            eligible=np.flatnonzero((separation*grid.resolution<=cfg['frontier_view_distance_m']) & (rx%step==0) & (ry%step==0))
            for i in eligible:
                p=grid.world([rx[i],ry[i]]);edge=grid.world([bx[nearest[i]],by[nearest[i]]])
                if grid.line_in_mask(p,edge,free):
                    sample={'viewpoint':p.tolist(),'known_free_path_length_m':float(dist[ry[i],rx[i]])}
                    if self.observe_full_state:
                        ex,ey=int(bx[nearest[i]]),int(by[nearest[i]])
                        sample.update(frontier_cluster=int(boundary_labels[ey,ex]),
                                      frontier_cell=[ex,ey],viewpoint_cell=[int(rx[i]),int(ry[i])])
                    sampled.append(sample)
        candidates=[]
        observed=[]
        for f in sampled:
            p=f['viewpoint']
            goal_distance=math.dist(p,goal);travel=f['known_free_path_length_m']
            extra=self.frontier_extra_cost(grid,dist,start,p)
            if math.dist(start,p)<1.0:
                if self.observe_full_state:
                    observed.append(dict(candidate_id=len(observed),target=p,
                        reachable=True,rejection_reason='within_1m_of_robot',
                        known_free_path_distance_m=travel,goal_distance_m=goal_distance,
                        frontier_cluster=f.get('frontier_cluster'),frontier_cell=f.get('frontier_cell'),
                        viewpoint_cell=f.get('viewpoint_cell'),selected=False))
                continue
            score=goal_distance+.3*travel+extra
            # Boundary bias may break close choices, but cannot impose an
            # arbitrarily expensive detour away from the mission target.
            boundary_cost=min(3.,self.boundary.penalty(p,clearance)) if self.boundary else 0.
            score+=boundary_cost
            candidates.append((score,p))
            if self.observe_full_state:
                observed.append(dict(candidate_id=len(observed),target=p,reachable=True,
                    rejection_reason=None,known_free_path_distance_m=travel,
                    goal_distance_m=goal_distance,frontier_cluster=f.get('frontier_cluster'),
                    frontier_cell=f.get('frontier_cell'),viewpoint_cell=f.get('viewpoint_cell'),
                    score_components={'goal_distance_m':goal_distance,
                                      'travel_weighted_cost':.3*travel,
                                      'extra_cost':extra,'boundary_cost':boundary_cost},
                    final_score=score,selected=False))
        if not candidates:
            self.target=None
            if self.observe_full_state:self.last_full_state={'candidates':observed,'previous_target':previous_target}
            return [],{'reason':'no_safe_frontier','frontiers':len(obs['frontiers'])}
        score,p=min(candidates,key=lambda x:x[0])
        # Bounded hysteresis only among freshly validated candidates. This
        # prevents flickering without retaining an obsolete exploration goal.
        if previous_target is not None:
            nearby=[item for item in candidates if math.dist(item[1],previous_target)<=grid.resolution*1.5]
            if nearby:
                retained=min(nearby,key=lambda item:item[0])
                if retained[0]<=score+3.:score,p=retained
        self.target=tuple(p);self.best=math.dist(start,p);self.progress_at=now
        if self.boundary:self.boundary.select(p)
        if self.observe_full_state:
            ranked=sorted((x for x in observed if x.get('rejection_reason') is None),key=lambda x:(x['final_score'],x['target']))
            for rank,item in enumerate(ranked,1):item['rank']=rank
            selected_record=min(ranked,key=lambda x:math.dist(x['target'],p))
            selected_record['selected']=True
            self.last_full_state={'candidates':observed,'previous_target':previous_target,
                                  'raw_minimum_target':min(candidates,key=lambda x:x[0])[1],
                                  'selected_target':p}
        return prefix+path_from_distances(grid,dist,p),{'reason':'current_frontier','target':p,'score':score,'candidate_count':len(candidates),'boundary_follow':self.boundary is not None,'boundary_side':self.boundary.side if self.boundary else 0}

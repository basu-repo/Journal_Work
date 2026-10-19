"""Sensor-derived grid and swept-clearance checks."""
import math
import numpy as np
from scipy.spatial import cKDTree

class Grid:
    def __init__(self,msg,start,goal,radius,cfg,hard_radius=None,free_scan=None,occupied_scan=None):
        self.cfg=cfg;self.res=cfg['planning_resolution']
        self.origin=np.array([msg.info.origin.position.x,msg.info.origin.position.y])
        q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.rot=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        local=np.array([self.local(start),self.local(goal)])
        lower=np.minimum(local.min(axis=0)-cfg['planning_margin'],[0,0])
        upper=np.maximum(local.max(axis=0)+cfg['planning_margin'],[msg.info.width*msg.info.resolution,msg.info.height*msg.info.resolution])
        self.lower=np.floor(lower/self.res)*self.res
        nx,ny=np.ceil((upper-self.lower)/self.res).astype(int)+1
        if nx*ny>cfg['planning_max_cells']:raise ValueError('planning_extent_exceeded')
        self.occupied=np.zeros((ny,nx),bool);self.known=np.zeros((ny,nx),bool)
        raw=np.asarray(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width).copy()
        self.live_cleared=0
        if free_scan is not None:self.live_cleared=self._reconcile_fresh_scan(raw,msg,free_scan)
        for condition,output in [(raw>=0,self.known),(raw>=cfg['occupied_threshold'],self.occupied)]:
            yy,xx=np.nonzero(condition)
            ix=np.floor(((xx+.5)*msg.info.resolution-self.lower[0])/self.res).astype(int)
            iy=np.floor(((yy+.5)*msg.info.resolution-self.lower[1])/self.res).astype(int)
            output[iy,ix]=True
        # Measure from original sensor-map cell centres. Binning occupied cells
        # into the planning grid first adds an unnecessary quantization offset.
        yy,xx=np.nonzero(raw>=cfg['occupied_threshold'])
        obstacles=np.column_stack(((xx+.5)*msg.info.resolution,(yy+.5)*msg.info.resolution))
        self.map_obstacle_tree=cKDTree(obstacles) if len(obstacles) else None
        self.live_obstacle_tree=None
        self.live_obstacle_cells=0
        if occupied_scan is not None:
            scan=np.asarray(occupied_scan,dtype=float).reshape(-1,2)
            scan=scan[np.isfinite(scan).all(axis=1)]
            scan=scan[np.linalg.norm(scan-np.asarray(start),axis=1)<=cfg.get(
                'live_scan_obstacle_range',cfg['live_scan_override_range'])]
            if len(scan):
                scan=(scan-self.origin)@self.rot
                cells=np.floor((scan-self.lower)/self.res).astype(int)
                inside=(cells[:,0]>=0)&(cells[:,1]>=0)&(cells[:,0]<nx)&(cells[:,1]<ny)
                scan=scan[inside]
                cells=np.unique(cells[inside],axis=0)
                self.live_obstacle_cells=len(cells)
                if len(cells):
                    self.occupied[cells[:,1],cells[:,0]]=True
                    self.known[cells[:,1],cells[:,0]]=True
                    # Keep measured hit positions. Cell centres can shift a
                    # nearby wall toward the robot by half a planning cell.
                    self.live_obstacle_tree=cKDTree(scan)
        cy,cx=np.indices((ny,nx))
        centres=np.column_stack((self.lower[0]+(cx.ravel()+.5)*self.res,
                                 self.lower[1]+(cy.ravel()+.5)*self.res))
        self.source_margin=math.sqrt(2)*msg.info.resolution/2
        self.distance=self.nearest_distance(centres).reshape(ny,nx)
        self.radius=radius
        self.hard_radius=radius if hard_radius is None else hard_radius
        self.blocked=self.distance<=radius
        self.source_cell_relaxed=False
    def allow_clear_start(self,start):
        """Open a coarse source cell only when the real robot pose is clear.

        The first graph edge still checks a swept segment from ``start``.
        This changes the planning copy, never the sensor map or obstacles.
        """
        x,y=self.cell(start)
        if self.free((x,y)) or not (0<=y<self.blocked.shape[0] and 0<=x<self.blocked.shape[1]):
            return False
        if float(self.nearest_distance(self.local(start)))<=self.radius:
            return False
        self.blocked[y,x]=False
        self.source_cell_relaxed=True
        return True
    def _reconcile_fresh_scan(self,raw,msg,scan):
        """Clear only map cells proved empty by adjacent current laser rays.

        This changes the planning copy, not the SLAM map. Near-field rollout
        collision checks still consume the unmodified live sensor returns.
        """
        yy,xx=np.nonzero(raw>=self.cfg['occupied_threshold'])
        if not len(xx):return 0
        cells=np.column_stack(((xx+.5)*msg.info.resolution,(yy+.5)*msg.info.resolution))
        world=cells@self.rot.T+self.origin
        origin=np.asarray(scan['origin'],dtype=float)
        offsets=world-origin
        distance=np.linalg.norm(offsets,axis=1)
        bearing=np.arctan2(offsets[:,1],offsets[:,0])-scan['yaw']
        bearing=(bearing+math.pi)%(2*math.pi)-math.pi
        beam=np.rint((bearing-scan['angle_min'])/scan['angle_increment']).astype(int)
        ranges=np.asarray(scan['ranges'],dtype=float)
        valid=(distance<=self.cfg['live_scan_override_range'])&(beam>=2)&(beam<len(ranges)-2)
        if not np.any(valid):return 0
        selected=np.flatnonzero(valid)
        neighboring=ranges[beam[selected,None]+np.arange(-2,3)[None,:]]
        neighboring=np.where(np.isposinf(neighboring),scan['range_max'],neighboring)
        clear=np.isfinite(neighboring).all(axis=1)&(neighboring.min(axis=1)>distance[selected]+self.cfg['live_scan_free_margin'])
        cleared=selected[clear]
        raw[yy[cleared],xx[cleared]]=0
        return len(cleared)
    def nearest_distance(self,local_points):
        # SLAM cells occupy an area; live LiDAR returns already give a point
        # location and must not inherit the SLAM cell-size uncertainty.
        points=np.asarray(local_points,dtype=float)
        shape=points.shape[:-1]
        result=np.full(shape,float('inf'))
        if self.map_obstacle_tree is not None:
            result=np.minimum(result,self.map_obstacle_tree.query(points)[0]-self.source_margin)
        if self.live_obstacle_tree is not None:
            result=np.minimum(result,self.live_obstacle_tree.query(points)[0])
        return result
    def local(self,p):return self.rot.T@(np.asarray(p)-self.origin)
    def cell(self,p):return tuple(np.floor((self.local(p)-self.lower)/self.res).astype(int))
    def xy(self,c):return tuple(self.origin+self.rot@(self.lower+(np.array(c)+.5)*self.res))
    def free(self,c):
        x,y=c;return 0<=y<self.blocked.shape[0] and 0<=x<self.blocked.shape[1] and not self.blocked[y,x]
    def line_clear(self,a,b):
        count=max(2,int(math.ceil(math.dist(a,b)/(self.res/3)))+1)
        return self.motion_clear(np.linspace(a,b,count))
    def motion_clear(self,points,radius=None):
        radius=self.radius if radius is None else radius
        points=np.asarray(points)
        if not len(points):return False
        local=(points-self.origin)@self.rot
        cells=np.floor((local-self.lower)/self.res).astype(int)
        if ((cells<0).any() or (cells[:,0]>=self.blocked.shape[1]).any()
                or (cells[:,1]>=self.blocked.shape[0]).any()):return False
        if self.map_obstacle_tree is None and self.live_obstacle_tree is None:return True
        distances=self.nearest_distance(local)
        sample_margin=max(np.linalg.norm(np.diff(points,axis=0),axis=1),default=0.)/2
        if np.min(distances)>radius+sample_margin:return True
        # A map update may introduce an inflation overlap. Permit only an
        # escape that retains physical clearance and never approaches it.
        if distances[0]>radius:return False
        return (np.min(distances)>self.hard_radius+sample_margin
                and np.all(np.diff(distances)>=-1e-9))

"""Local free-ray evidence for the planning copy, never the published map.

Occupied cells are retained. Unknown is cleared only in the local patch when
five neighboring current rays all reach beyond a cell plus a margin. This is
sensor inverse mapping, not prior world knowledge or goal-based clearing.
"""
import math
import numpy as np
from exploration_memory.core import MapGrid

def supplement_free(grid,scan,extent,margin):
    pad=int(math.ceil(extent/grid.resolution))
    raw=np.pad(grid.data,pad,constant_values=-1).copy()
    origin=np.array(grid.origin[:2])-grid.rotation@np.array([pad,pad])*grid.resolution
    out=MapGrid(raw,grid.resolution,(*origin,grid.origin[2]))
    yy,xx=np.indices(raw.shape)
    world=out.world(np.column_stack([xx.ravel(),yy.ravel()]))
    delta=world-np.array(scan['origin']);distance=np.linalg.norm(delta,axis=1)
    angle=np.arctan2(delta[:,1],delta[:,0])-scan['yaw'];angle=(angle+math.pi)%(2*math.pi)-math.pi
    beam=np.rint((angle-scan['angle_min'])/scan['angle_increment']).astype(int)
    ranges=np.asarray(scan['ranges']);selected=np.flatnonzero((raw.ravel()==-1)&(distance<=extent)&(beam>=2)&(beam<len(ranges)-2))
    r=ranges[beam[selected,None]+np.arange(-2,3)]
    r=np.where(np.isposinf(r),scan['range_max'],r)
    good=np.isfinite(r).all(axis=1)&(r.min(axis=1)>distance[selected]+margin)
    raw.ravel()[selected[good]]=0
    return MapGrid(raw,grid.resolution,(*origin,grid.origin[2]))

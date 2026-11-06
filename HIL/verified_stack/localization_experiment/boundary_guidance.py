"""Optional obstacle-boundary bias for reachable frontier selection.

Only observed occupancy is used. This heuristic does not guarantee maze
completion and never authorizes motion or relaxes collision clearance.
"""
import math
import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

class BoundaryPreference:
    def __init__(self):
        self.goal=None;self.hit=None;self.direction=None;self.side=0;self.tree=None

    def prepare(self,grid,start,goal):
        if self.goal!=tuple(goal):
            self.__init__();self.goal=tuple(goal)
        delta=np.asarray(goal)-start
        samples=np.linspace(start,goal,max(2,int(math.ceil(np.linalg.norm(delta)/grid.resolution))+1))
        labels,_=ndimage.label(grid.data>=65,structure=np.ones((3,3)))
        label=0
        for p in samples:
            c=grid.cell(p)
            if grid.inside(c) and labels[c[1],c[0]]:
                label=labels[c[1],c[0]];break
        if label:
            yy,xx=np.nonzero(labels==label)
            self.tree=cKDTree(grid.world(np.column_stack([xx,yy])))
            if self.hit is None:
                self.hit=np.asarray(start,dtype=float);self.direction=delta/max(np.linalg.norm(delta),1e-9)
        # Keep the last observed blocking boundary while going around its end.
        return self.tree is not None

    def lateral(self,p):
        v=np.asarray(p)-self.hit
        return self.direction[0]*v[1]-self.direction[1]*v[0]

    def penalty(self,p,clearance):
        if self.tree is None:return 0.
        distance=float(self.tree.query(p)[0])
        # Prefer frontiers close to the boundary, but retain unchanged safe-space masks.
        penalty=4.*max(0.,distance-(clearance+1.))
        if self.side and self.side*self.lateral(p)<-1.:
            penalty+=12.
        return penalty

    def select(self,p):
        if self.tree is not None and not self.side:
            lateral=self.lateral(p)
            if abs(lateral)>.5:self.side=1 if lateral>0 else -1

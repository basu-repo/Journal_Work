"""Optional execution-horizon fallback; no collision/map modifications."""
import math
class HorizonFallback:
    def __init__(self,stall_seconds=10.,short_m=1.,release_m=1.):
        self.stall_seconds=stall_seconds;self.short_m=short_m;self.release_m=release_m
        self.since=None;self.anchor=None;self.short=False
    def update(self,now,xy,search_waiting,executing):
        if self.short:
            if math.dist(xy,self.anchor)>=self.release_m:
                self.short=False;self.anchor=None;self.since=None;return 'restored'
            return None
        if search_waiting and not executing:
            if self.since is None:self.since=now
            elif now-self.since>=self.stall_seconds:
                self.short=True;self.anchor=tuple(xy);self.since=None;return 'reduced'
        else:self.since=None
        return None
    def limit(self,base):return min(base,self.short_m) if self.short else base

"""Opt-in short GNSS acquisition and dead-reckoning cycle; no ROS dependencies."""
import math

WAYPOINT_TRANSITION_STATES = frozenset(('WAYPOINT_SCAN', 'WAYPOINT_ALIGN', 'WAYPOINT_RESCAN'))


def acquire_holds_supervisor(state):
    """Let a waypoint handoff finish while GNSS acquires; hold normal driving."""
    return state not in WAYPOINT_TRANSITION_STATES


class GnssCycle:
    def __init__(self, settings, start_acquiring=True):
        self.cfg=settings
        self.state='ACQUIRE' if start_acquiring else 'COAST'
        self.started_at=None
        self.coast_started_at=None
        self.start_xy=None
        self.last_xy=None
        self.travel_m=0.
        self.last_coast_distance=0.
        self.path_sequence=None
        self.no_path_since=None
        self.fix_count=0
        self.segment_id=0
        self.last_reason='startup'

    def coast(self, now, xy, path_sequence, reason='target_zone_exit'):
        """Disable acquisition and begin a fresh dead-reckoning interval."""
        self.state='COAST';self.coast_started_at=now
        self.start_xy=tuple(xy);self.last_xy=tuple(xy);self.travel_m=0.
        self.path_sequence=path_sequence;self.no_path_since=None
        self.last_reason=reason
        return reason

    def note_fix(self):
        if self.state=='ACQUIRE':self.fix_count+=1

    def acquire(self, now, reason):
        if self.state=='ACQUIRE':return None
        self.state='ACQUIRE';self.started_at=now;self.fix_count=0
        self.start_xy=None;self.last_xy=None;self.path_sequence=None
        self.no_path_since=None;self.last_reason=reason
        return reason

    def distance(self,xy):
        return self.travel_m+math.dist(self.last_xy,xy) if self.last_xy is not None else 0.

    def step(self,now,xy,path_sequence,has_path,sigma):
        if self.started_at is None:self.started_at=now
        if self.state=='ACQUIRE':
            if (self.fix_count>=self.cfg['min_fixes'] and
                    now-self.started_at>=self.cfg['acquire_s'] and
                    math.isfinite(sigma) and sigma<self.cfg['sigma_limit']):
                self.state='COAST';self.coast_started_at=now
                self.start_xy=tuple(xy);self.last_xy=tuple(xy);self.travel_m=0.
                self.path_sequence=path_sequence
                self.no_path_since=None
                self.segment_id+=1;self.last_reason='segment_locked'
                return 'segment_locked'
            return None
        self.travel_m=self.distance(xy);self.last_xy=tuple(xy)
        # Ordinary lattice replans revise the route frequently. Keep coasting
        # while a checked path exists; acquire at a stop, distance/time limit,
        # or uncertainty limit instead of at every path publication.
        reason=None
        if has_path:self.no_path_since=None
        elif self.no_path_since is None:self.no_path_since=now
        if (self.no_path_since is not None and
                now-self.no_path_since>=self.cfg.get('route_unavailable_grace_s',0.)):
            reason='route_unavailable'
        elif not math.isfinite(sigma) or sigma>=self.cfg['sigma_limit']:reason='uncertainty_limit'
        elif self.distance(xy)>=self.cfg['segment_m']:reason='segment_complete'
        elif now-self.coast_started_at>=self.cfg['max_coast_s']:reason='coast_timeout'
        if reason:
            self.last_coast_distance=self.travel_m
            return self.acquire(now,reason)
        return None

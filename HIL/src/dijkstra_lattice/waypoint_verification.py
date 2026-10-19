"""Provisional waypoint arrival followed by a fresh GNSS pose check."""
import math


class WaypointVerifier:
    def __init__(self, settings):
        self.settings = settings
        self.pending = None

    def begin(self, now, target, estimated_xy, estimated_distance):
        self.pending = dict(started_at=now, target=target,
                            estimated_xy=tuple(estimated_xy),
                            estimated_distance=float(estimated_distance))

    def check(self, now, target, corrected_xy, fixes, acquiring,
              localization_ready, sigma):
        pending = self.pending
        if pending is None or pending['target'] != target:
            return 'idle', None
        if now-pending['started_at'] >= self.settings['waypoint_gnss_timeout_s']:
            return 'timeout', None
        if (not acquiring or fixes < self.settings['gnss_segments']['min_fixes'] or
                now-pending['started_at'] < self.settings['gnss_segments']['acquire_s'] or
                not localization_ready or not math.isfinite(sigma) or
                sigma >= self.settings['gnss_segments']['sigma_limit']):
            return 'waiting', None
        distance = math.dist(corrected_xy, target)
        self.pending = None
        outcome = ('accepted' if distance <= self.settings['goal_tolerance']-
                   self.settings['arrival_reserve'] else 'rejected')
        return outcome, distance

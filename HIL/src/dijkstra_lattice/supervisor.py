"""Independent D* recovery supervisor with checked reverse and rotation.

The caller retains sensor obstacles throughout recovery; it never clears real
obstacles to force a reverse. Shared localization health gates all commands.
"""
import math

from .core import escape_command, rotation_command, navigation_radius, turn_away_direction, wrap




class Supervisor:
    def __init__(self, robot):
        self.robot=robot
        self.settings=robot['settings']
        self.clearance_radius=lambda sigma:navigation_radius(sigma,robot,self.settings)
        self.state = 'GOTO_WAYPOINT'
        self.blocked_since = None
        self.recovery_start = None
        self.recovery_started_at = None
        self.recovery_count = 0
        self.recovery_distance = 0.0
        self.rotation_start = None
        self.rotation_direction = 1
        self.last_recovery_event = None
        self.waypoint_transition_started_at = None
        self.waypoint_transition_target = None

    def _start_recovery(self, now, local_pose):
        self.state = 'RECOVERY_BACKUP'
        self.recovery_start = (local_pose.x, local_pose.y, local_pose.yaw)
        self.recovery_started_at = now
        self.recovery_distance = 0.0
        self.recovery_count += 1
        self.last_recovery_event = 'backup_entered'

    def _start_rotation(self, now, local_pose):
        self.state = 'RECOVERY_ROTATE'
        self.rotation_start = local_pose.yaw
        self.recovery_started_at = now
        self.last_recovery_event = 'rotation_entered'

    def step(self, now, pose, local_pose, localization_mode, sigma, points,
             nearest, proposed, planner_state, high_slip=False, obstacle_sector=None,
             next_target=None, latest_lidar_stamp=None, trajectory_clear=None):
        self.last_recovery_event = None
        if self.state == 'RECOVERY_FAILED':
            return 0.0, 0.0, False
        uncertainty_near_obstacle = (localization_mode != 'anchored' and sigma > 1.0
                                     and nearest is not None and nearest < self.clearance_radius(sigma) + 0.5)
        if localization_mode != 'anchored' and (sigma > 2.0 or uncertainty_near_obstacle):
            self.state = 'RE_ACQUIRE_GNSS'
            return 0.0, 0.0, False
        if self.state == 'RE_ACQUIRE_GNSS':
            # The navigation node holds this supervisor while the periodic
            # receiver is in ACQUIRE.  Once it calls us again in COAST, a
            # corrected low-uncertainty estimate means reacquisition finished.
            # Requiring the literal "anchored" label here permanently latched
            # periodic operation in this state after every successful fix.
            if sigma >= 1.0:
                return 0.0, 0.0, False
            self.state = 'GOTO_WAYPOINT'
            self.blocked_since = None


        # An arrival does not immediately become a new forward command.  The
        # robot first takes a scan after it has stopped, aligns only if the
        # turning envelope is clear, then requires one more post-turn scan.
        # This prevents a waypoint handoff from carrying stale perception or
        # the previous leg's heading into the new leg.
        if self.state == 'WAYPOINT_SCAN':
            if (now - self.waypoint_transition_started_at < self.settings['waypoint_scan_wait'] or
                    latest_lidar_stamp is None or
                    latest_lidar_stamp <= self.waypoint_transition_started_at):
                return 0.0, 0.0, False
            target = self.waypoint_transition_target or next_target
            if target is None:
                self.state = 'GOTO_WAYPOINT'
                self.last_recovery_event = 'waypoint_scan_finished'
                return 0.0, 0.0, True
            heading_error = wrap(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
            if abs(heading_error) <= math.radians(self.settings['waypoint_heading_degrees']):
                self.state = 'WAYPOINT_RESCAN'
                self.waypoint_transition_started_at = now
                self.last_recovery_event = 'waypoint_already_aligned'
                return 0.0, 0.0, False
            self.state = 'WAYPOINT_ALIGN'
            self.last_recovery_event = 'waypoint_scan_finished'
            return 0.0, 0.0, False
        if self.state == 'WAYPOINT_ALIGN':
            target = self.waypoint_transition_target or next_target
            if target is None:
                self.state = 'GOTO_WAYPOINT'
                return 0.0, 0.0, True
            heading_error = wrap(math.atan2(target[1] - pose.y, target[0] - pose.x) - pose.yaw)
            if abs(heading_error) <= math.radians(self.settings['waypoint_heading_degrees']):
                self.state = 'WAYPOINT_RESCAN'
                self.waypoint_transition_started_at = now
                self.last_recovery_event = 'waypoint_aligned'
                return 0.0, 0.0, False
            if now - self.waypoint_transition_started_at > self.settings['waypoint_align_timeout']:
                # Do not turn blindly after a timeout. Normal local planning
                # will select a safe detour from the fresh grid on the next tick.
                self.state = 'GOTO_WAYPOINT'
                self.last_recovery_event = 'waypoint_align_timeout'
                return 0.0, 0.0, True
            direction = 1 if heading_error > 0 else -1
            v, w, result = rotation_command(pose, points, sigma, direction, self.robot, trajectory_clear)
            if result != 'rotating':
                self.state = 'GOTO_WAYPOINT'
                self.last_recovery_event = 'waypoint_turning_space_blocked'
                return 0.0, 0.0, True
            return v, w, False
        if self.state == 'WAYPOINT_RESCAN':
            if (now - self.waypoint_transition_started_at < self.settings['waypoint_rescan_wait'] or
                    latest_lidar_stamp is None or
                    latest_lidar_stamp <= self.waypoint_transition_started_at):
                return 0.0, 0.0, False
            self.state = 'GOTO_WAYPOINT'
            self.last_recovery_event = 'waypoint_rescan_finished'
            return 0.0, 0.0, True

        if self.state == 'RECOVERY_ROTATE':
            turned = abs(wrap(local_pose.yaw - self.rotation_start))
            if turned >= math.radians(self.settings['recovery_turn_angle_degrees']):
                self.state = 'GOTO_WAYPOINT'
                self.blocked_since = None
                self.last_recovery_event = 'rotation_finished'
                return 0.0, 0.0, True
            if now - self.recovery_started_at > self.settings['recovery_turn_timeout']:
                self.state = 'RECOVERY_FAILED'
                self.last_recovery_event = 'rotation_timeout'
                return 0.0, 0.0, False
            v, w, result = rotation_command(pose, points, sigma, self.rotation_direction, self.robot, trajectory_clear)
            if result != 'rotating':
                self.state = 'RECOVERY_FAILED'
                self.last_recovery_event = 'rotation_blocked'
                return 0.0, 0.0, False
            return v, w, False
        if self.state == 'RECOVERY_BACKUP':
            dx = local_pose.x - self.recovery_start[0]
            dy = local_pose.y - self.recovery_start[1]
            travel = -(dx * math.cos(self.recovery_start[2]) + dy * math.sin(self.recovery_start[2]))
            self.recovery_distance = max(self.recovery_distance, travel)
            forward_available=(planner_state=='tracking' and proposed[0]>0 and travel>=self.settings['backup_min_distance'])
            if travel >= self.settings['backup_distance'] or forward_available:
                self.state = 'GOTO_WAYPOINT'
                self.blocked_since = None
                self.recovery_start = None
                self.last_recovery_event = 'backup_finished'
                return 0.0, 0.0, True
            if now - self.recovery_started_at > self.settings['backup_timeout']:
                self._start_rotation(now, local_pose)
                return 0.0, 0.0, False
            v, w, result = escape_command(pose, points, sigma, self.robot, trajectory_clear)
            if result != 'escaping':
                self._start_rotation(now, local_pose)
                return 0.0, 0.0, False
            return (max(v, -0.1) if high_slip else v), w, False
        if planner_state in ('blocked', 'stalled_stop'):
            if self.blocked_since is None:
                self.blocked_since = now
            if now - self.blocked_since >= self.settings['recovery_wait']:
                if self.recovery_count >= self.settings['recovery_retries']:
                    self.state = 'RECOVERY_FAILED'
                    self.last_recovery_event = 'retry_limit'
                    return 0.0, 0.0, False
                self.rotation_direction = turn_away_direction(obstacle_sector)
                if obstacle_sector is None:
                    self.rotation_direction = 1 if self.recovery_count % 2 == 0 else -1
                self._start_recovery(now, local_pose)
                return 0.0, 0.0, False
            self.state = 'OBSTACLE_AVOID'
            return 0.0, 0.0, False

        self.blocked_since = None
        # This is a behaviour/status boundary only.  The rollout's hard
        # chance-constrained clearance remains independent and larger when
        # localization uncertainty grows.
        self.state = ('OBSTACLE_AVOID' if nearest is not None and
                      nearest < self.settings['obstacle_avoid_distance'] else 'GOTO_WAYPOINT')
        return proposed[0], proposed[1], False

    def waypoint_advanced(self, now=None, next_target=None):
        self.state = 'WAYPOINT_SCAN' if now is not None and next_target is not None else 'GOTO_WAYPOINT'
        self.blocked_since = None
        self.recovery_count = 0
        self.recovery_distance = 0.0
        self.waypoint_transition_started_at = now
        self.waypoint_transition_target = next_target

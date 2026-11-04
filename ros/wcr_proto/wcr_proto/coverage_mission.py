"""Coverage mission for the skid-steer prototype: the dam sim's CoverageMission (wcr_webots) with
lane changes the prototype can drive.

The 4WIS design robot crabs sideways between lanes. The prototype cannot crab, and the twin shows
it cannot turn on the spot on a steep face either: its wheelbase (200 mm) is longer than its track
(190 mm), so the tyres scrub sideways and it slides down instead of turning (turning back up from
about 45 deg does not happen at all). So, like the original, it keeps facing up the face (up-lanes
forwards, down-lanes in reverse), and it changes lanes with a gentle S-curve while driving into the
next lane (at most ~20-30 deg of heading) instead of a sideways move. Same for the way home.
The lane plan, Nav2 legs, coverage grid, detections and report are the original node's.
"""
import math

import numpy as np
import rclpy
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path as PathMsg
from wcr_webots.coverage_mission import CoverageMission

REACH = 0.17          # m from the base to the farthest robot part (board, ESC leads, lidar, prop disc)
SHIFT_LEN = 2.5       # S-curve length per metre of sideways shift (smoothstep: max slope 1.5 / 2.5 -> 31 deg)
MIN_SHIFT_LEN = 0.5   # m: shortest S-curve (a 12 cm lane change -> ~20 deg)
OVERSHOOT = 0.3       # m driven past each lane end (when the face and the map allow it)


class ProtoCoverage(CoverageMission):
    def robot_blocked(self, s, t):
        """Would the robot base at (s, t) come within `clearance` of a mapped obstacle?"""
        for it in self.doc['items']:
            if not it.get('mapped'):
                continue
            ls, lt = it['size']
            if it['type'] == 'bush':
                cs = it['s'] + 0.2 * ls
                if math.hypot(cs - s, it['t'] - t) < 0.5 * ls + REACH + self.clearance:
                    return True
            else:
                half_t = 0.1 if it['type'] == 'pipe' else lt / 2
                if abs(it['s'] - s) < ls / 2 + REACH + self.clearance and \
                        abs(it['t'] - t) < half_t + REACH + self.clearance:
                    return True
        return False

    def build_legs(self):
        """As the original, but a change to another lane is driven as part of that lane (S-curve
        from where the robot is), not as a separate sideways move."""
        b, _ = self.lookup('base_link')
        legs, cur = [('move', (b[0], b[1]), self.route[0])], self.route[0]
        pts = self.route[1:]
        (fs0, fs1) = self.doc['area']['s']
        for k in range(0, len(pts) - 1, 2):
            a, e = pts[k], pts[k + 1]
            if abs(a[1] - cur[1]) < 0.02:                         # same lane, next segment: move along it
                legs.append(('move', cur, a))
            # run past the lane end so the next lane change (S-curve) happens mostly outside the area
            d = 1.0 if e[0] > a[0] else -1.0
            ext = (e[0] + d * OVERSHOOT, e[1])
            if fs0 + 0.3 < ext[0] < fs1 - 0.3 and not self.robot_blocked(*ext):
                e = ext
            legs.append(('lane', a, e))
            cur = e
        legs.append(('home', cur, self.home))
        self.legs, self.leg_i = legs, 0

    def straight_path(self, a, e):
        """Path from a to e facing up the face (yaw 0, driven in reverse when going down). A sideways
        offset is taken as a smooth S-curve at the start of the path."""
        ds, dt = e[0] - a[0], e[1] - a[1]
        n = max(2, int(max(abs(ds), abs(dt)) / 0.05) + 1)
        u = np.linspace(0.0, 1.0, n)
        s = a[0] + ds * u
        if abs(dt) < 0.02:
            t = a[1] + dt * u
        else:
            L = min(abs(ds), max(MIN_SHIFT_LEN, SHIFT_LEN * abs(dt))) if abs(ds) > 0.05 else 0.0
            x = np.clip(np.abs(s - a[0]) / L, 0.0, 1.0) if L > 0 else u
            t = a[1] + dt * x * x * (3 - 2 * x)
        path = PathMsg()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = [self.pose(si, ti) for si, ti in zip(s, t)]
        return path

    def send(self):
        """The way home is a FollowPath S-curve too (NavigateToPose would ask for spot turns)."""
        if self.leg_i < len(self.legs) and self.legs[self.leg_i][0] == 'home':
            _, a, e = self.legs[self.leg_i]
            b, _ = self.lookup('base_link')
            here = (b[0], b[1])
            length = float(np.hypot(e[0] - here[0], e[1] - here[1]))
            self.leg_limit = 25.0 + 4.0 * length / max(self.speed, 0.05)
            self.t_leg = self.now()
            goal = FollowPath.Goal(path=self.straight_path(here, e), controller_id='FollowPath',
                                   goal_checker_id='general_goal_checker')
            self.follow.send_goal_async(goal).add_done_callback(self.on_accepted)
            return
        super().send()


def main():
    rclpy.init()
    node = ProtoCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

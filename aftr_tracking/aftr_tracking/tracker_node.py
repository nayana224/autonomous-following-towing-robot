# Copyright (c) 2026 Autonomous Following and Towing Robot Contributors
"""LaserScan based nearest-person following node."""

import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, TransformStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration as DurationMsg
import numpy as np
import math
from std_msgs.msg import String


class NearestPersonFollower(Node):
    """Track the nearest leg-pair target and publish bounded base commands."""

    def __init__(self):
        super().__init__("person_follower")

        self.scan_timeout_markers_cleared = False
        self.target_pub = self.create_publisher(Point, "/target_person", 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/person_markers", 10)
        self.state_pub = self.create_publisher(
            String, "/follow_state", 10
        )
        self.prev_stamp = None

        self.mode = "SEARCH"
        self.search_kp_yaw = 0.9
        self.search_w_max = 0.8

        self.follow_fov_deg = 80.0
        self.follow_fov_rad = math.radians(self.follow_fov_deg)

        self.last_scan_time = self.get_clock().now()
        self.scan_timeout_s = 0.25

        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0

        self.soft_stop_active = False

        self.a_lin = 0.6
        self.a_ang = 1.2

        self.watchdog_dt = 0.05
        self.watchdog_timer = self.create_timer(self.watchdog_dt, self.watchdog_cb)

        self.a_lin_follow = 0.8
        self.a_ang_follow = 2.0

        self.stop_buffer = 0.03

        self.d_ref = (
            0.47  # Preserve the existing following setpoint.
        )
        self.kp_dist = 0.85
        self.kp_yaw = 1.25
        self.v_max_follow = 3.0
        self.w_max_follow = 1.5
        self.prev_w = 0.0
        self.max_dw = 0.12

        self.a_stop = (
            1.0
        )
        self.a_ang_stop = 1.0
        self.d_stop_margin = 0.04
        self.ttc_thresh = 0.6

        self.prev_d = None

        self.d_max_neighbor = 0.07
        self.s_min = 0.005
        self.s_max = 0.3
        self.d_leg_th = 0.5
        self.frame_id = "laser"

        self.fov_deg = 50.0
        self.fov_rad = math.radians(self.fov_deg)

        self.laser_yaw_offset = 3.141592

        self.leg_spread_min = 0.03
        self.leg_spread_max = 0.20

        self.locked_target = None
        self.lost_frames = 0
        self.max_lost_frames = 16
        self.follow_match_threshold = 0.55

        self.candidate_count = 0
        self.required_frames = 20
        self.detection_dist_limit = 2.0

        self.block_half_width = 0.15
        self.block_front_margin = 0.20  # Exclude the immediate robot footprint from corridor checks.
        self.block_target_margin = 0.30  # Exclude points close to the target from corridor checks.
        self.block_stop_distance = 0.10  # Stop immediately for an obstacle inside this distance.

        self.blocked_count = 0
        self.blocked_required_frames = 2

        self.pause_by_obstacle = False

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile_sensor_data
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_marker_count = 0

        self.get_logger().info("TF 발행 기능이 포함된 추적 노드가 시작되었습니다.")

    def scan_callback(self, msg: LaserScan):
        """Convert each scan to candidates and update follow or safe-stop commands."""
        self.scan_timeout_markers_cleared = False
        self.last_scan_time = self.get_clock().now()

        points = []
        angle = msg.angle_min
        for dist in msg.ranges:
            if (
                msg.range_min < dist < msg.range_max
                and not math.isinf(dist)
                and not math.isnan(dist)
            ):
                points.append([dist * math.cos(angle), dist * math.sin(angle)])
            angle += msg.angle_increment

        # Measure elapsed time from the previous scan for acceleration limits.
        now = self.get_clock().now()
        if self.prev_stamp is None:
            dt = self.watchdog_dt
        else:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9

        dt = max(0.01, min(0.2, dt))
        self.prev_stamp = now

        # Decelerate through the slew limiter when a scan has no usable data.
        if not points:
            target = self.locked_target
            self.publish_visualization([], target)
            self.prev_d = None

            v_des, w_des = 0.0, 0.0
            self.soft_stop_active = True
            v_cmd, w_cmd = self.apply_slew(v_des, w_des, dt)
            self.publish_cmd(v_cmd, w_cmd)
            self.publish_follow_state()
            return

        clusters = self.perform_clustering(points)

        humans = self.detect_humans(clusters)

        humans_fov = self.filter_humans_by_fov(humans)

        v_des, w_des = 0.0, 0.0
        d = None
        target = None

        if self.mode == "SEARCH":
            target = self.search_target(humans_fov)

            if target is None:
                self.soft_stop_active = True
                v_des = 0.0
                w_des = 0.0

                v_cmd, w_cmd = self.apply_slew(v_des, w_des, dt)
                self.publish_cmd(v_cmd, w_cmd)
                self.publish_visualization(humans_fov, None)
                self.publish_follow_state()
                return

        elif self.mode == "FOLLOW":
            target = self.update_locked_target(humans_fov)

        if target is not None:
            point_msg = Point()
            point_msg.x, point_msg.y = float(target[0]), float(target[1])
            self.target_pub.publish(point_msg)

            self.broadcast_target_tf(target)

            lx = float(target[0])
            ly = float(target[1])
            d = math.hypot(lx, ly)

            # Check the corridor between robot and target for obstacles.
            blocked, obs_dist = self.between_robot_and_target(points, target)

            if blocked:
                self.blocked_count += 1
            else:
                self.blocked_count = 0

            if self.blocked_count >= self.blocked_required_frames:
                self.pause_by_obstacle = True

                if self.locked_target is not None:
                    self.lost_frames = 0

                v_des = 0.0
                w_des = 0.0
                self.soft_stop_active = True

                if obs_dist is not None:
                    self.get_logger().warn(
                        f"사람-로봇 사이 장애물 감지 -> 정지 | obstacle_dist={obs_dist:.3f} m"
                    )
                else:
                    self.get_logger().warn("사람-로봇 사이 장애물 감지 -> 정지")

                v_cmd, w_cmd = self.apply_slew(v_des, w_des, dt)
                self.publish_cmd(v_cmd, w_cmd)
                self.publish_visualization(humans_fov, target)

                self.publish_follow_state("OBSTACLE")

                return
            else:
                self.pause_by_obstacle = False

            # Reset distance history after a target jump to avoid a false TTC stop.
            if self.prev_d is not None:
                if abs(d - self.prev_d) > 0.8:
                    self.prev_d = None

            yaw_err = math.atan2(ly, lx) + self.laser_yaw_offset
            yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi

            if abs(yaw_err) < math.radians(3.0):
                yaw_err = 0.0

            if d <= self.d_ref:
                v_des = 0.0
            else:
                v_des = self.kp_dist * (d - self.d_ref)

            # Reduce forward speed when the target moves far off axis.
            if abs(yaw_err) > math.radians(50):
                v_des *= 0.2

            w_des = -self.kp_yaw * yaw_err

            # Prioritize deceleration over rotation at close range.
            if d < (self.d_ref + 0.15):
                w_des *= 0.75

            # The extra buffer prevents command oscillation near the setpoint.
            if d < (self.d_ref + self.stop_buffer):
                v_des = 0.0
                if abs(yaw_err) < math.radians(5.0):
                    w_des = 0.0

            v_des = max(0.0, min(self.v_max_follow, v_des))
            w_des = max(-self.w_max_follow, min(self.w_max_follow, w_des))

            # Cap speed by the available stopping distance.
            d_remain = max(0.0, d - (self.d_ref + self.d_stop_margin))
            v_safe = math.sqrt(2.0 * self.a_stop * d_remain)
            v_safe = max(0.0, min(self.v_max_follow, v_safe))
            v_des = min(
                v_des, v_safe
            )

            # Brake on a short time-to-collision estimate.
            if self.prev_d is not None:
                v_closing = (self.prev_d - d) / max(
                    dt, 1e-3
                )
            else:
                v_closing = 0.0
            self.prev_d = d

            d_to_ref = max(0.0, d - self.d_ref)
            if v_closing > 0.2:
                ttc = d_to_ref / v_closing
                if ttc < self.ttc_thresh:
                    v_des = 0.0
                    w_des = 0.0
            v_des = max(0.0, min(self.v_max_follow, v_des))
            w_des = max(-self.w_max_follow, min(self.w_max_follow, w_des))

            self.soft_stop_active = False
        else:
            # Decelerate to a stop when the target is lost.
            self.soft_stop_active = True
            self.prev_w = 0.0
            self.prev_d = None

        v_cmd, w_cmd = self.apply_slew(v_des, w_des, dt)
        self.publish_cmd(v_cmd, w_cmd)

        if target is not None:
            self.get_logger().info(
                f"타겟 추적 중:  x={lx:.3f}, y={ly:.3f}, d={d:.3f}  | "
                f"현재 속도={v_cmd:.3f}, 현재 각속도={w_cmd:.3f}"
            )

        self.publish_visualization(humans_fov, target)

        self.publish_follow_state()

    def publish_follow_state(self, state=None):
        """Publish tracking state for centralized voice and LED feedback."""
        state_msg = String()
        state_msg.data = str(self.mode if state is None else state)
        self.state_pub.publish(state_msg)

    # Reject targets blocked by an obstacle in the robot-target corridor.
    def between_robot_and_target(self, points, target):
        """Report a blocking LiDAR cluster inside the robot-target corridor."""
        if target is None or not points:
            return False, None

        tx = float(target[0])
        ty = float(target[1])

        target_dist = math.hypot(tx, ty)
        if target_dist < 1e-6:
            return False, None

        ux = tx / target_dist
        uy = ty / target_dist

        nearest_obs_dist = None
        hit_count = 0

        for p in points:
            px = float(p[0])
            py = float(p[1])

            proj = px * ux + py * uy

            if proj < self.block_front_margin:
                continue

            # Exclude points close to the target from corridor checks.
            if proj > (target_dist - self.block_target_margin):
                continue

            perp = abs(px * uy - py * ux)

            if perp <= self.block_half_width:
                obs_dist = math.hypot(px, py)
                hit_count += 1

                if nearest_obs_dist is None or obs_dist < nearest_obs_dist:
                    nearest_obs_dist = obs_dist

        if hit_count >= 3:
            return True, nearest_obs_dist

        return False, None

    def filter_humans_by_fov(self, humans):
        """Use the narrower search field of view until a target is locked."""
        filtered = []

        if self.mode == "FOLLOW":
            fov_limit = self.follow_fov_rad
        else:
            fov_limit = self.fov_rad

        for h in humans:
            x = float(h[0])
            y = float(h[1])

            ang = math.atan2(y, x)
            ang = ang + self.laser_yaw_offset
            ang = (ang + math.pi) % (2.0 * math.pi) - math.pi

            if abs(ang) <= fov_limit:
                filtered.append(h)

        return filtered

    def apply_slew(self, v_des, w_des, dt):
        """Bound command changes, allowing faster deceleration for a safety stop."""
        dt = max(dt, 1e-3)

        # Limit forward acceleration and deceleration separately.
        dv = v_des - self.last_cmd_v
        if dv > 0.0:
            dv_max = self.a_lin_follow * dt
        else:
            dv_max = self.a_stop * dt  # Allow faster deceleration for a safety stop.

        v_cmd = self.last_cmd_v + max(-dv_max, min(dv_max, dv))

        dw = w_des - self.last_cmd_w
        if dw > 0.0:
            dw_max = self.a_ang_follow * dt
        else:
            dw_max = self.a_ang_stop * dt

        w_cmd = self.last_cmd_w + max(-dw_max, min(dw_max, dw))

        v_cmd = max(0.0, min(self.v_max_follow, v_cmd))
        w_cmd = max(-self.w_max_follow, min(self.w_max_follow, w_cmd))

        if abs(v_cmd) < 0.05 and self.mode != "SEARCH":
            w_cmd = 0.0

        return float(v_cmd), float(w_cmd)

    def publish_cmd(self, v, w):
        """Publish a velocity command and retain it for timeout deceleration."""
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        self.last_cmd_v = float(v)
        self.last_cmd_w = float(w)

    def broadcast_target_tf(self, target):
        """Publish the selected target relative to the LiDAR frame."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = "target_person"

        x = float(target[0])
        y = float(target[1])

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        yaw = math.atan2(y, x) + self.laser_yaw_offset
        yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
        qx, qy, qz, qw = self.yaw_to_quat(yaw)

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def yaw_to_quat(self, yaw):
        """Return quaternion components for a planar target heading."""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def perform_clustering(self, points):
        """Group adjacent scan points, including clusters across the scan seam."""
        clusters = []
        if not points:
            return clusters

        current_cluster = [points[0]]
        for i in range(1, len(points)):
            dist = math.hypot(
                points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]
            )
            if dist < self.d_max_neighbor:
                current_cluster.append(points[i])
            else:
                if len(current_cluster) >= 2:
                    clusters.append(current_cluster)
                current_cluster = [points[i]]

        if len(current_cluster) >= 2:
            clusters.append(current_cluster)

        if len(clusters) >= 2:
            first = clusters[0][0]
            last = clusters[-1][-1]
            wrap_dist = math.hypot(first[0] - last[0], first[1] - last[1])
            if wrap_dist < self.d_max_neighbor:
                merged = clusters[-1] + clusters[0]
                clusters = clusters[1:-1] + [merged]

        return clusters

    def detect_humans(self, clusters):
        """Pair leg-shaped clusters into person candidates."""
        leg_candidates = self.detect_leg_candidates(clusters)
        return self.pair_leg_candidates(leg_candidates)

    def detect_leg_candidates(self, clusters, spread_max=None):
        """Return cluster centers that satisfy the existing leg geometry."""
        leg_candidates = []
        effective_spread_max = (
            self.leg_spread_max if spread_max is None else float(spread_max)
        )

        for c in clusters:
            np_c = np.array(c, dtype=np.float32)

            width = float(np.max(np_c[:, 0]) - np.min(np_c[:, 0]))
            height = float(np.max(np_c[:, 1]) - np.min(np_c[:, 1]))
            area = width * height

            if not (self.s_min <= area <= self.s_max):
                continue

            mean = np.mean(np_c, axis=0)
            centered = np_c - mean

            cov = np.cov(centered.T)
            if cov.shape != (2, 2):
                continue

            eigvals = np.linalg.eigvals(cov)
            eigvals = np.sort(np.real(eigvals))[::-1]
            major_spread = float(math.sqrt(max(eigvals[0], 0.0)) * 2.0)

            if not (self.leg_spread_min <= major_spread <= effective_spread_max):
                continue

            leg_candidates.append(mean)

        return leg_candidates

    def pair_leg_candidates(self, leg_candidates):
        """Greedily pair leg candidates using the established distance rule."""
        humans = []
        used_indices = set()

        for i in range(len(leg_candidates)):
            if i in used_indices:
                continue
            for j in range(i + 1, len(leg_candidates)):
                if j in used_indices:
                    continue

                dist = math.hypot(
                    float(leg_candidates[i][0] - leg_candidates[j][0]),
                    float(leg_candidates[i][1] - leg_candidates[j][1]),
                )

                if dist <= self.d_leg_th:
                    humans.append((leg_candidates[i] + leg_candidates[j]) / 2.0)
                    used_indices.add(i)
                    used_indices.add(j)
                    break

        return humans

    def search_target(self, humans):
        """Lock the nearest candidate after the required confirmation frames."""
        valid_candidates = [
            h
            for h in humans
            if 0.3 <= math.hypot(h[0], h[1]) <= self.detection_dist_limit
        ]

        if not valid_candidates:
            self.candidate_count = 0
            return None

        current_nearest = min(valid_candidates, key=lambda h: math.hypot(h[0], h[1]))

        self.candidate_count += 1

        if self.candidate_count >= self.required_frames:
            self.locked_target = current_nearest
            self.lost_frames = 0
            self.candidate_count = 0
            self.mode = "FOLLOW"
            return self.locked_target

        return None

    def update_locked_target(self, humans):
        """Keep or update the selected target while FOLLOW mode is active."""
        if not humans:
            if self.locked_target is not None:
                if not self.pause_by_obstacle:
                    self.lost_frames += 1

                if self.lost_frames > self.max_lost_frames:
                    self.locked_target = None
                    self.candidate_count = 0
                    self.mode = "SEARCH"
                    return None

                return self.locked_target

            self.mode = "SEARCH"
            self.candidate_count = 0
            return None

        if self.locked_target is not None:
            best_match = min(
                humans,
                key=lambda h: math.hypot(
                    h[0] - self.locked_target[0], h[1] - self.locked_target[1]
                ),
            )

            dist = math.hypot(
                best_match[0] - self.locked_target[0],
                best_match[1] - self.locked_target[1],
            )

            if dist < self.follow_match_threshold:
                self.locked_target = best_match
                self.lost_frames = 0
                return best_match
            else:
                if not self.pause_by_obstacle:
                    self.lost_frames += 1

                if self.lost_frames > self.max_lost_frames:
                    self.locked_target = None
                    self.candidate_count = 0
                    self.mode = "SEARCH"
                    return None

                return self.locked_target

        self.mode = "SEARCH"
        return None

    def publish_visualization(self, humans, target):
        """Publish candidate markers and delete IDs no longer present."""
        marker_array = MarkerArray()

        for i, h in enumerate(humans):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.lifetime = DurationMsg(sec=0, nanosec=100_000_000)

            m.pose.position.x, m.pose.position.y = float(h[0]), float(h[1])
            m.pose.position.z = 0.5
            m.scale.x, m.scale.y, m.scale.z = 0.25, 0.25, 1.0

            is_target = target is not None and np.allclose(h, target)
            m.color.a = 1.0
            if is_target:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            elif self.candidate_count > 0:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0

            marker_array.markers.append(m)

        for j in range(len(humans), self.prev_marker_count):
            dm = Marker()
            dm.header.frame_id = self.frame_id
            dm.header.stamp = self.get_clock().now().to_msg()
            dm.id = j
            dm.action = Marker.DELETE
            marker_array.markers.append(dm)

        self.prev_marker_count = len(humans)

        self.marker_pub.publish(marker_array)

    def watchdog_cb(self):
        """Decelerate after scan timeout and clear stale markers once."""
        now = self.get_clock().now()
        dt_since_scan = (now - self.last_scan_time).nanoseconds * 1e-9
        scan_timeout = dt_since_scan > self.scan_timeout_s

        # Only the scan-loss watchdog may command velocity after scan updates stop.
        if not scan_timeout:
            return

        dt = self.watchdog_dt

        v = self.last_cmd_v
        if v > 0.0:
            v = max(0.0, v - self.a_stop * dt)
        elif v < 0.0:
            v = min(0.0, v + self.a_stop * dt)

        w = self.last_cmd_w
        if w > 0.0:
            w = max(0.0, w - self.a_ang_stop * dt)
        elif w < 0.0:
            w = min(0.0, w + self.a_ang_stop * dt)

        self.last_cmd_v = v
        self.last_cmd_w = w

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

        # Clear stale RViz markers once after scan input stops.
        if scan_timeout and (not self.scan_timeout_markers_cleared):
            marker_array = MarkerArray()
            for j in range(self.prev_marker_count):
                dm = Marker()
                dm.header.frame_id = self.frame_id
                dm.header.stamp = self.get_clock().now().to_msg()
                dm.id = j
                dm.action = Marker.DELETE
                marker_array.markers.append(dm)
            self.marker_pub.publish(marker_array)
            self.prev_marker_count = 0
            self.scan_timeout_markers_cleared = True


def main():
    """Run the follower and release ROS resources on exit."""
    node = None
    try:
        rclpy.init()
        node = NearestPersonFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("person_follower interrupted by SIGINT")
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"person_follower crashed: {exc}")
        else:
            print(f"person_follower crashed before startup completed: {exc}")
        traceback.print_exc()
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                node.get_logger().warn(
                    f"failed to destroy person_follower node cleanly: {exc}"
                )

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
            except Exception as exc:
                if node is not None:
                    node.get_logger().warn(
                        f"failed to shut down rclpy cleanly: {exc}"
                    )
                else:
                    print(f"failed to shut down rclpy cleanly: {exc}")


if __name__ == "__main__":
    main()

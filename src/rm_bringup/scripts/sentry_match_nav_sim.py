#!/usr/bin/env python3
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path as NavPath
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rm_interfaces.msg import AutoaimTargetStatus, GimbalStatus, SentrySimInput
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def yaw_to_quat(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def as_float(data, key, default=0.0):
    try:
        return float(data.get(key, default))
    except (TypeError, ValueError):
        return default


def as_bool(data, key, default=False):
    value = data.get(key, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


class SentryMatchNavSim(Node):
    def __init__(self):
        super().__init__("sentry_match_nav_sim")
        self.frame_id = self.declare_parameter("frame_id", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "gimbal_yaw_fake").value
        self.action_name = self.declare_parameter("navigate_action_name", "navigate_to_pose").value
        self.goals_file = self.declare_parameter("goals_file", "").value
        self.scenario_path = self.declare_parameter("scenario_path", "").value
        self.speed_mps = float(self.declare_parameter("speed_mps", 1.2).value)
        self.update_hz = max(2.0, float(self.declare_parameter("update_hz", 20.0).value))
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.08).value)
        self.robot_x = float(self.declare_parameter("initial_x", 0.0).value)
        self.robot_y = float(self.declare_parameter("initial_y", 0.0).value)
        self.robot_yaw = float(self.declare_parameter("initial_yaw", 2.15).value)
        self.goal_x = self.robot_x
        self.goal_y = self.robot_y
        self.goal_yaw = self.robot_yaw
        self.active = False
        self.trail = []
        self.start_time_sec = time.monotonic()
        self.last_hp = None
        self.damage_flash_until = 0.0

        self.latest_status = None
        self.latest_sim_input = None
        self.scenario = self.load_yaml(self.scenario_path)
        self.goals = self.load_goals(self.goals_file)
        self.phase_ends = self.compute_phase_ends(self.scenario.get("phases", []))
        self.visual_cfg = self.scenario.get("visualization", {})

        self.tf_broadcaster = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(NavPath, "/sentry_match_sim/path", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/sentry_match_sim/markers", 10)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.target_pub = self.create_publisher(
            AutoaimTargetStatus, "/autoaim/target_status", sensor_qos)
        self.status_sub = self.create_subscription(
            GimbalStatus, "/gimbal_status", self.on_status, sensor_qos)
        self.sim_input_sub = self.create_subscription(
            SentrySimInput, "/sentry_bt/sim_input", self.on_sim_input, sensor_qos)

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            self.action_name,
            execute_callback=self.execute_goal,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.timer = self.create_timer(1.0 / self.update_hz, self.publish_visuals)
        self.get_logger().info(
            f"fake NavigateToPose server ready on {self.action_name}; "
            f"robot={self.frame_id}->{self.robot_frame}, goals={len(self.goals)}, "
            f"scenario={self.scenario_path or 'none'}"
        )

    def load_yaml(self, path):
        if not path:
            return {}
        try:
            return yaml.safe_load(Path(path).read_text()) or {}
        except Exception as exc:
            self.get_logger().warn(f"failed to load yaml {path}: {exc}")
            return {}

    def load_goals(self, path):
        data = self.load_yaml(path)
        goals = []
        for name, goal in data.get("goals", {}).items():
            goals.append(
                {
                    "name": name,
                    "id": int(goal.get("id", 0)),
                    "x": float(goal.get("x", 0.0)),
                    "y": float(goal.get("y", 0.0)),
                    "yaw": float(goal.get("yaw", 0.0)),
                }
            )
        return goals

    @staticmethod
    def compute_phase_ends(phases):
        total = 0.0
        ends = []
        for phase in phases:
            total += max(0.0, float(phase.get("duration_ms", 0))) / 1000.0
            ends.append(total)
        return ends

    def current_phase_index(self):
        if not self.phase_ends:
            return 0
        elapsed = time.monotonic() - self.start_time_sec
        wrapped = elapsed % max(self.phase_ends[-1], 0.001)
        for index, end_sec in enumerate(self.phase_ends):
            if wrapped <= end_sec:
                return index
        return len(self.phase_ends) - 1

    def current_phase(self):
        phases = self.scenario.get("phases", [])
        if not phases:
            return {}
        return phases[self.current_phase_index()]

    def on_status(self, msg):
        if self.last_hp is not None and msg.current_hp < self.last_hp:
            self.damage_flash_until = time.monotonic() + 2.0
        self.last_hp = int(msg.current_hp)
        self.latest_status = msg

    def on_sim_input(self, msg):
        self.latest_sim_input = msg

    def goal_callback(self, goal_request):
        self.goal_x = float(goal_request.pose.pose.position.x)
        self.goal_y = float(goal_request.pose.pose.position.y)
        self.goal_yaw = quat_to_yaw(goal_request.pose.pose.orientation)
        self.active = True
        self.trail = [self.current_pose_msg()]
        self.get_logger().info(
            f"accepted fake nav goal x={self.goal_x:.3f} y={self.goal_y:.3f} "
            f"yaw={self.goal_yaw:.2f}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.active = False
        return CancelResponse.ACCEPT

    def execute_goal(self, goal_handle):
        start_time = self.get_clock().now()
        period = 1.0 / self.update_hz
        result = NavigateToPose.Result()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.active = False
                goal_handle.canceled()
                return result

            distance = self.step_robot(period)
            feedback = NavigateToPose.Feedback()
            feedback.current_pose = self.current_pose_msg()
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            feedback.navigation_time.sec = int(elapsed)
            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)
            self.publish_visuals()

            if distance <= self.goal_tolerance:
                self.robot_x = self.goal_x
                self.robot_y = self.goal_y
                self.robot_yaw = self.goal_yaw
                self.active = False
                goal_handle.succeed()
                self.publish_visuals()
                return result

            time.sleep(period)

        self.active = False
        goal_handle.abort()
        return result

    def active_obstacles(self):
        visual = self.visual_cfg
        phase_visual = self.current_phase().get("visual", {})
        obstacles = []
        obstacles.extend(visual.get("teammates", []))
        obstacles.extend(visual.get("obstacles", []))
        obstacles.extend(phase_visual.get("teammates", []))
        obstacles.extend(phase_visual.get("obstacles", []))
        return obstacles

    def active_enemies(self):
        visual = self.visual_cfg
        phase_visual = self.current_phase().get("visual", {})
        enemies = []
        enemies.extend(visual.get("enemies", []))
        enemies.extend(phase_visual.get("enemies", []))
        if self.latest_sim_input and self.latest_sim_input.enemy_in_view:
            explicit = phase_visual.get("active_enemy", {})
            if explicit:
                enemies.append(explicit)
            else:
                distance = max(0.5, float(self.latest_sim_input.enemy_distance_m))
                enemies.append({
                    "name": "AUTOAIM_TARGET",
                    "x": self.robot_x + math.cos(self.robot_yaw) * distance,
                    "y": self.robot_y + math.sin(self.robot_yaw) * distance,
                    "radius": 0.32,
                    "active": True,
                })
        return enemies

    def step_robot(self, dt):
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        distance = math.hypot(dx, dy)
        if distance <= self.goal_tolerance:
            return distance

        vx = dx / distance
        vy = dy / distance
        for obstacle in self.active_obstacles():
            ox = as_float(obstacle, "x")
            oy = as_float(obstacle, "y")
            radius = as_float(obstacle, "radius", 0.45) + 0.55
            rx = self.robot_x - ox
            ry = self.robot_y - oy
            od = max(0.001, math.hypot(rx, ry))
            ahead = ((ox - self.robot_x) * vx + (oy - self.robot_y) * vy) > -0.2
            if ahead and od < radius:
                strength = (radius - od) / radius
                vx += (rx / od) * strength * 1.8
                vy += (ry / od) * strength * 1.8

        norm = max(0.001, math.hypot(vx, vy))
        vx /= norm
        vy /= norm
        step = min(distance, self.speed_mps * dt)
        self.robot_x += vx * step
        self.robot_y += vy * step
        self.robot_yaw = math.atan2(vy, vx)
        if not self.trail or math.hypot(
            self.robot_x - self.trail[-1].pose.position.x,
            self.robot_y - self.trail[-1].pose.position.y,
        ) > 0.08:
            self.trail.append(self.current_pose_msg())
            self.trail = self.trail[-240:]
        return max(0.0, distance - step)

    def current_pose_msg(self):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = self.robot_x
        pose.pose.position.y = self.robot_y
        qx, qy, qz, qw = yaw_to_quat(self.robot_yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def publish_target_status(self):
        msg = AutoaimTargetStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        sim = self.latest_sim_input
        if sim and sim.enemy_in_view:
            msg.has_target = True
            msg.tracking = sim.enemy_confidence >= 0.55
            msg.temp_lost = False
            msg.fire_ready = sim.enemy_confidence >= 0.75 and sim.enemy_distance_m <= 7.0
            msg.target_distance = float(sim.enemy_distance_m)
            msg.yaw_error_deg = 0.0
            msg.pitch_error_deg = 0.0
            msg.aim_source = 1
            msg.reason = "match_sim_target"
        else:
            msg.has_target = False
            msg.reason = "match_sim_no_target"
        self.target_pub.publish(msg)

    def publish_visuals(self):
        self.publish_target_status()
        now = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.robot_frame
        tf.transform.translation.x = self.robot_x
        tf.transform.translation.y = self.robot_y
        qx, qy, qz, qw = yaw_to_quat(self.robot_yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        path = NavPath()
        path.header.stamp = now
        path.header.frame_id = self.frame_id
        path.poses = list(self.trail[-160:]) or [self.current_pose_msg()]
        goal = PoseStamped()
        goal.header.stamp = now
        goal.header.frame_id = self.frame_id
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        qx, qy, qz, qw = yaw_to_quat(self.goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        path.poses.append(goal)
        self.path_pub.publish(path)

        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        markers.markers.append(self.robot_marker(now))
        markers.markers.append(self.current_goal_marker(now))
        markers.markers.append(self.status_text_marker(now))
        if time.monotonic() <= self.damage_flash_until:
            markers.markers.append(self.damage_marker(now))
        for index, goal_data in enumerate(self.goals):
            markers.markers.append(self.goal_marker(now, goal_data, index + 10))
        marker_id = 100
        for item in self.active_obstacles():
            markers.markers.extend(self.obstacle_markers(now, item, marker_id))
            marker_id += 2
        for item in self.active_enemies():
            markers.markers.extend(self.enemy_markers(now, item, marker_id))
            marker_id += 3
        self.marker_pub.publish(markers)

    def robot_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "robot"
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        qx, qy, qz, qw = yaw_to_quat(self.robot_yaw)
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        marker.scale.x = 0.50
        marker.scale.y = 0.50
        marker.scale.z = 0.50
        dead = self.latest_status is not None and self.latest_status.current_hp == 0
        marker.color.r = 0.9 if dead else 0.1
        marker.color.g = 0.1 if dead else 0.8
        marker.color.b = 0.1 if dead else 1.0
        marker.color.a = 0.88
        return marker

    def current_goal_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "current_goal"
        marker.id = 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        marker.color.r = 1.0
        marker.color.g = 0.8
        marker.color.b = 0.1
        marker.color.a = 0.9
        return marker

    def status_text_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "status_text"
        marker.id = 3
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.95
        marker.scale.z = 0.22
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        hp = "HP ?"
        if self.latest_status:
            hp = f"HP {self.latest_status.current_hp}/{self.latest_status.maximum_hp}"
            if self.latest_status.current_hp == 0:
                hp += " DEAD"
        enemy = "enemy -"
        if self.latest_sim_input and self.latest_sim_input.enemy_in_view:
            enemy = f"enemy {self.latest_sim_input.enemy_distance_m:.1f}m"
        marker.text = f"{hp}\n{enemy}\nphase {self.current_phase_index() + 1}"
        return marker

    def damage_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "damage"
        marker.id = 4
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.robot_x
        marker.pose.position.y = self.robot_y
        marker.pose.position.z = 0.75
        marker.scale.x = 0.9
        marker.scale.y = 0.9
        marker.scale.z = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.35
        return marker

    def goal_marker(self, stamp, goal_data, marker_id):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "sentry_goals"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = goal_data["x"]
        marker.pose.position.y = goal_data["y"]
        marker.pose.position.z = 0.35
        marker.scale.z = 0.22
        marker.color.r = 0.9
        marker.color.g = 0.9
        marker.color.b = 0.9
        marker.color.a = 0.95
        marker.text = f'{goal_data["name"]} ({goal_data["id"]})'
        return marker

    def obstacle_markers(self, stamp, item, marker_id):
        radius = as_float(item, "radius", 0.45)
        color = item.get("color", "ally")
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = self.frame_id
        body.ns = "allies_obstacles"
        body.id = marker_id
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = as_float(item, "x")
        body.pose.position.y = as_float(item, "y")
        body.pose.position.z = 0.25
        body.scale.x = radius * 2.0
        body.scale.y = radius * 2.0
        body.scale.z = 0.5
        body.color.r = 0.1 if color == "ally" else 0.55
        body.color.g = 0.35 if color == "ally" else 0.55
        body.color.b = 1.0 if color == "ally" else 0.55
        body.color.a = 0.65

        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = self.frame_id
        label.ns = "allies_obstacles_text"
        label.id = marker_id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = body.pose.position.x
        label.pose.position.y = body.pose.position.y
        label.pose.position.z = 0.85
        label.scale.z = 0.2
        label.color.r = 0.7
        label.color.g = 0.85
        label.color.b = 1.0
        label.color.a = 0.95
        label.text = item.get("name", "ALLY/OBSTACLE")
        return [body, label]

    def enemy_markers(self, stamp, item, marker_id):
        active = as_bool(item, "active", True)
        radius = as_float(item, "radius", 0.32)
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = self.frame_id
        body.ns = "enemies"
        body.id = marker_id
        body.type = Marker.SPHERE
        body.action = Marker.ADD
        body.pose.position.x = as_float(item, "x")
        body.pose.position.y = as_float(item, "y")
        body.pose.position.z = 0.35
        body.scale.x = radius * 2.0
        body.scale.y = radius * 2.0
        body.scale.z = 0.55
        body.color.r = 1.0
        body.color.g = 0.12
        body.color.b = 0.08
        body.color.a = 0.9 if active else 0.35

        text = Marker()
        text.header.stamp = stamp
        text.header.frame_id = self.frame_id
        text.ns = "enemy_text"
        text.id = marker_id + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = body.pose.position.x
        text.pose.position.y = body.pose.position.y
        text.pose.position.z = 0.95
        text.scale.z = 0.22
        text.color.r = 1.0
        text.color.g = 0.45
        text.color.b = 0.35
        text.color.a = 0.95
        text.text = item.get("name", "ENEMY")

        beam = Marker()
        beam.header.stamp = stamp
        beam.header.frame_id = self.frame_id
        beam.ns = "enemy_beam"
        beam.id = marker_id + 2
        beam.type = Marker.LINE_STRIP
        beam.action = Marker.ADD
        beam.scale.x = 0.04
        beam.color.r = 1.0
        beam.color.g = 0.0
        beam.color.b = 0.0
        beam.color.a = 0.7 if active else 0.0
        beam.points = [
            Point(x=self.robot_x, y=self.robot_y, z=0.35),
            Point(x=body.pose.position.x, y=body.pose.position.y, z=0.35),
        ]
        return [body, text, beam]


def main():
    rclpy.init()
    node = SentryMatchNavSim()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

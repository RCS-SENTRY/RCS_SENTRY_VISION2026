#!/usr/bin/env python3

import math
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rm_interfaces.msg import GimbalStatus


class InitialPoseManager(Node):
    def __init__(self) -> None:
        super().__init__('rm_initial_pose_manager')

        self.declare_parameter('gimbal_status_topic', '/gimbal_status')
        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_on_startup', True)
        self.declare_parameter('startup_delay_sec', 3.0)
        self.declare_parameter('publish_on_game_progress', True)
        self.declare_parameter('game_progress_trigger', 3)
        self.declare_parameter('publish_burst_count', 5)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('fallback_robot_id', 7)
        self.declare_parameter('red_start_pose', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('blue_start_pose', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('covariance_xy', 0.25)
        self.declare_parameter('covariance_z', 0.10)
        self.declare_parameter('covariance_yaw_deg', 10.0)

        self.gimbal_status_topic = self.get_parameter('gimbal_status_topic').value
        self.initialpose_topic = self.get_parameter('initialpose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_on_startup = bool(self.get_parameter('publish_on_startup').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self.publish_on_game_progress = bool(self.get_parameter('publish_on_game_progress').value)
        self.game_progress_trigger = int(self.get_parameter('game_progress_trigger').value)
        self.publish_burst_count = int(self.get_parameter('publish_burst_count').value)
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self.fallback_robot_id = int(self.get_parameter('fallback_robot_id').value)
        self.red_start_pose = self._read_pose_param('red_start_pose')
        self.blue_start_pose = self._read_pose_param('blue_start_pose')
        self.covariance_xy = float(self.get_parameter('covariance_xy').value)
        self.covariance_z = float(self.get_parameter('covariance_z').value)
        self.covariance_yaw_deg = float(self.get_parameter('covariance_yaw_deg').value)

        self.current_robot_id = self.fallback_robot_id
        self.last_game_progress = None
        self.pending_publish_reason = ''
        self.remaining_burst = 0
        self.pending_pose: Tuple[float, float, float, float] = self._pose_for_robot_id(self.current_robot_id)
        self.burst_timer = None

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.initialpose_topic, 10)
        self.status_sub = self.create_subscription(
            GimbalStatus,
            self.gimbal_status_topic,
            self.on_gimbal_status,
            10)

        self.startup_timer = None
        if self.publish_on_startup:
            self.startup_timer = self.create_timer(self.startup_delay_sec, self.on_startup_timer)

        self.get_logger().info(
            f'InitialPoseManager ready. startup_publish={self.publish_on_startup} '
            f'game_progress_trigger={self.game_progress_trigger} fallback_robot_id={self.fallback_robot_id}')

    def _read_pose_param(self, name: str) -> Tuple[float, float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 4:
            raise RuntimeError(f'{name} must contain exactly 4 numbers: [x, y, z, yaw_deg]')
        return float(values[0]), float(values[1]), float(values[2]), float(values[3])

    @staticmethod
    def _team_from_robot_id(robot_id: int) -> str:
        if 1 <= robot_id <= 11:
            return 'red'
        if 101 <= robot_id <= 111:
            return 'blue'
        return 'unknown'

    def _pose_for_robot_id(self, robot_id: int) -> Tuple[float, float, float, float]:
        team = self._team_from_robot_id(robot_id)
        if team == 'red':
            return self.red_start_pose
        if team == 'blue':
            return self.blue_start_pose
        return self._pose_for_robot_id(self.fallback_robot_id) if robot_id != self.fallback_robot_id else self.red_start_pose

    def on_startup_timer(self) -> None:
        if self.startup_timer is not None:
            self.startup_timer.cancel()
            self.startup_timer = None
        self.schedule_publish('startup')

    def on_gimbal_status(self, msg: GimbalStatus) -> None:
        if self._team_from_robot_id(msg.robot_id) != 'unknown':
            if msg.robot_id != self.current_robot_id:
                self.get_logger().info(
                    f'Referee robot_id updated: {self.current_robot_id} -> {msg.robot_id}')
            self.current_robot_id = int(msg.robot_id)

        if self.publish_on_game_progress:
            if self.last_game_progress != self.game_progress_trigger and msg.game_progress == self.game_progress_trigger:
                self.schedule_publish(f'game_progress={self.game_progress_trigger}')
            self.last_game_progress = int(msg.game_progress)

    def schedule_publish(self, reason: str) -> None:
        self.pending_publish_reason = reason
        self.pending_pose = self._pose_for_robot_id(self.current_robot_id)
        self.remaining_burst = max(1, self.publish_burst_count)

        if self.burst_timer is not None:
            self.burst_timer.cancel()

        self.burst_timer = self.create_timer(self.publish_period_sec, self.on_burst_timer)
        self.get_logger().info(
            'Scheduling initial pose publish: '
            f'reason={reason} robot_id={self.current_robot_id} pose={self.pending_pose}')

    def on_burst_timer(self) -> None:
        if self.remaining_burst <= 0:
            if self.burst_timer is not None:
                self.burst_timer.cancel()
                self.burst_timer = None
            return

        x, y, z, yaw_deg = self.pending_pose
        yaw_rad = math.radians(yaw_deg)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        cov = [0.0] * 36
        cov[0] = self.covariance_xy * self.covariance_xy
        cov[7] = self.covariance_xy * self.covariance_xy
        cov[14] = self.covariance_z * self.covariance_z
        cov[35] = math.radians(self.covariance_yaw_deg) ** 2
        msg.pose.covariance = cov

        self.initial_pose_pub.publish(msg)
        self.remaining_burst -= 1

        if self.remaining_burst == 0:
            self.get_logger().info(
                f'Published initial pose burst complete ({self.pending_publish_reason}).')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

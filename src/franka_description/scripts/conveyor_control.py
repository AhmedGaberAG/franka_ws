#!/usr/bin/env python3
"""Stop the conveyor when a cube reaches the pick station.

The cubes are moved by TrajectoryFollower plugins.  This node watches the
Gazebo pose stream, finds the frontmost cube that is still on the belt and
publishes a shared pause command to all cube followers.  When that cube is
lifted or removed, the next cube is allowed to advance.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


class ConveyorControl(Node):
    def __init__(self) -> None:
        super().__init__('conveyor_control')

        self.declare_parameter('pose_topic', '/world/warehouse/dynamic_pose/info')
        self.declare_parameter('pause_topic', '/conveyor/pause')
        self.declare_parameter('target_x', 0.70)
        self.declare_parameter('stop_tolerance', 0.03)
        self.declare_parameter('resume_tolerance', 0.055)
        self.declare_parameter('belt_x_min', 0.55)
        self.declare_parameter('belt_z_min', 0.55)
        self.declare_parameter('belt_z_max', 0.75)
        self.declare_parameter('belt_y_max', 0.22)
        self.declare_parameter('rate_hz', 20.0)

        pose_topic = self.get_parameter('pose_topic').value
        pause_topic = self.get_parameter('pause_topic').value
        self.target_x = float(self.get_parameter('target_x').value)
        self.stop_tolerance = float(self.get_parameter('stop_tolerance').value)
        self.resume_tolerance = float(
            self.get_parameter('resume_tolerance').value)
        self.belt_x_min = float(self.get_parameter('belt_x_min').value)
        self.belt_z_min = float(self.get_parameter('belt_z_min').value)
        self.belt_z_max = float(self.get_parameter('belt_z_max').value)
        self.belt_y_max = float(self.get_parameter('belt_y_max').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.positions = {}
        self.stopped = False
        self.last_published = 0.0

        self.pause_pub = self.create_publisher(Bool, pause_topic, 10)
        self.pose_sub = self.create_subscription(
            TFMessage, pose_topic, self._on_poses, 10)
        self.timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._update)

        self.get_logger().info(
            f'Conveyor control: target x={self.target_x:.2f} m, '
            f'poses={pose_topic}, pause={pause_topic}')

    def _on_poses(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            name = transform.child_frame_id
            if not name.startswith('cube_'):
                continue
            translation = transform.transform.translation
            self.positions[name] = (
                translation.x, translation.y, translation.z)

    def _on_belt(self, pose) -> bool:
        x, y, z = pose
        return (
            x >= self.belt_x_min
            and self.belt_z_min <= z <= self.belt_z_max
            and abs(y) <= self.belt_y_max
        )

    def _update(self) -> None:
        on_belt = {
            name: pose for name, pose in self.positions.items()
            if self._on_belt(pose)
        }
        front = min(on_belt.items(), key=lambda item: item[1][0]) \
            if on_belt else None

        if front is None:
            self._set_stopped(False, 'belt is clear')
        else:
            name, (x, _, _) = front
            if self.stopped:
                if x > self.target_x + self.resume_tolerance:
                    self._set_stopped(False, f'{name} left the pick station')
            elif x <= self.target_x + self.stop_tolerance:
                self._set_stopped(True, f'{name} reached x={x:.3f} m')

        now = time.monotonic()
        if now - self.last_published >= 0.5:
            self._publish()

    def _set_stopped(self, stopped: bool, reason: str) -> None:
        if stopped == self.stopped:
            return
        self.stopped = stopped
        if stopped:
            self.get_logger().info(f'Conveyor stopped: {reason}')
        else:
            self.get_logger().info(f'Conveyor running: {reason}')
        self._publish()

    def _publish(self) -> None:
        msg = Bool()
        msg.data = self.stopped
        self.pause_pub.publish(msg)
        self.last_published = time.monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConveyorControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

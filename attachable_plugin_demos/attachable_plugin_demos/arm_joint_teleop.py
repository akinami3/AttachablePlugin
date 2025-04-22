#!/usr/bin/env python3
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ArmJointTeleop(Node):
    def __init__(self):
        super().__init__('arm_joint_teleop')
        self.pub = self.create_publisher(
            JointTrajectory, 'arm_robot/joint_trajectory', 10
        )

        # List of joint names (example for a 6-joint robot)
        self.joint_names = [
            'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6',
        ]
        # Current angles of each joint (in radians)
        # self.joint_positions = [0.0, 1.90, 0.8, 0.0, 0.0, 0.0]
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Index of the currently selected joint (0 to 5)
        self.selected_joint = 0
        # Angle step size (in radians)
        self.step = 0.05

        self._publish_current()
        print("Press 1-6 to select joint, z/x to increase/decrease angle, Ctrl-C to exit.")


    def run(self):
        """Loop while accepting key inputs. Exit with Ctrl-C."""
        # Switch terminal to raw mode
        old_attrs = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin)
        try:
            while rclpy.ok():
                if self._kbhit():
                    c = sys.stdin.read(1)
                    if c in ('\x03', '\x04'):  # Ctrl-C or Ctrl-D
                        break
                    self._on_key(c)
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)

    def _kbhit(self):
        """Return True if there is data in stdin"""
        dr, _, _ = select.select([sys.stdin], [], [], 0.1)
        return bool(dr)

    def _on_key(self, c):
        """Handle key press events"""
        # Select joint number
        if c in ['1','2','3','4','5','6']:
            self.selected_joint = int(c) - 1
            return

        # Increase or decrease angle
        if c == 'z':
            self.joint_positions[self.selected_joint] += self.step
        elif c == 'x':
            self.joint_positions[self.selected_joint] -= self.step
        else:
            # Ignore other keys
            return

        # Send updated values
        self._publish_current()

    def _publish_current(self):
        """Publish the current joint_positions as a trajectory with a single point"""
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        pt = JointTrajectoryPoint()
        pt.positions = list(self.joint_positions)
        # Reach the target in 0.1 seconds (arbitrary)
        pt.time_from_start = Duration(sec=0, nanosec=int(0.1 * 1e9))
        traj.points.append(pt)

        self.pub.publish(traj)
        # Round to 3 decimal places
        self.joint_positions = [round(pos, 3) for pos in self.joint_positions]
        # Output to log
        print(f"Published: {self.joint_positions}", end="\r\n")


def main(args=None):
    rclpy.init(args=args)
    node = ArmJointTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

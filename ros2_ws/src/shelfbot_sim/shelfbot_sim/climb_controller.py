#!/usr/bin/env python3
"""
climb_controller.py — state-machine controller for shelf approach, hooking,
                       climbing, and bottle grabbing.

STATE MACHINE:
  IDLE         → waiting for a /climb_controller/start command
  APPROACH     → drive toward shelf face using /cmd_vel, guided by rail_pose
  ALIGN        → fine-adjust lateral position until rail is centred
  HOOK         → rotate clamp_arm_joint to hook the jaw onto the rail
  CLIMB        → extend climb_joint (prismatic) to target level height
  REACH        → rotate manip_shoulder_joint to bring arm to bottle height
  GRASP        → close gripper_finger joints to grab bottle
  RETREAT      → retract manip arm after grasp
  DESCEND      → retract climb_joint back to 0
  DONE         → publish success, return to IDLE

Topics subscribed:
  /shelf_detector/rail_pose  (geometry_msgs/PoseStamped) — rail location
  /shelf_detector/status     (std_msgs/String)            — detection state
  /climb_controller/start    (std_msgs/Float32)           — target shelf level (m)
  /joint_states              (sensor_msgs/JointState)     — current joint positions

Topics published:
  /cmd_vel                       (geometry_msgs/Twist)     — base drive
  /climb_controller/joint_cmd    (sensor_msgs/JointState)  — joint position targets
  /climb_controller/state        (std_msgs/String)         — current FSM state
"""

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float32


class ClimbController(Node):

    # Joint name → index mapping (populated from /joint_states)
    JOINTS = [
        'clamp_arm_joint',
        'climb_joint',
        'clamp_jaw_joint',
        'manip_shoulder_joint',
        'gripper_joint',
        'gripper_finger_left_joint',
        'gripper_finger_right_joint',
        'camera_pan_joint',
    ]

    def __init__(self):
        super().__init__('climb_controller')

        self._state     = 'IDLE'
        self._target_level = 0.55    # default: shelf level 1 (0.55 m)
        self._rail_pose = None
        self._detector_status = 'SEARCHING'
        self._joint_pos = {}

        # Subscriptions
        self.create_subscription(PoseStamped, '/shelf_detector/rail_pose',
                                 self._rail_cb, 10)
        self.create_subscription(String, '/shelf_detector/status',
                                 self._status_cb, 10)
        self.create_subscription(Float32, '/climb_controller/start',
                                 self._start_cb, 10)
        self.create_subscription(JointState, '/joint_states',
                                 self._joint_state_cb, 10)

        # Publishers
        self._vel_pub   = self.create_publisher(Twist,      '/cmd_vel', 10)
        self._joint_pub = self.create_publisher(JointState, '/climb_controller/joint_cmd', 10)
        self._state_pub = self.create_publisher(String,     '/climb_controller/state', 10)

        # 20 Hz control loop
        self.create_timer(0.05, self._step)
        self.get_logger().info('ClimbController ready — publish to /climb_controller/start '
                               'with target level height in metres')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _rail_cb(self, msg: PoseStamped):
        self._rail_pose = msg

    def _status_cb(self, msg: String):
        self._detector_status = msg.data

    def _start_cb(self, msg: Float32):
        self._target_level = float(msg.data)
        if self._state == 'IDLE':
            self.get_logger().info(f'Starting climb to level {self._target_level:.2f} m')
            self._state = 'APPROACH'

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos

    # ── Main step ─────────────────────────────────────────────────────────────
    def _step(self):
        self._publish_state(self._state)

        if   self._state == 'IDLE':     self._do_idle()
        elif self._state == 'APPROACH': self._do_approach()
        elif self._state == 'ALIGN':    self._do_align()
        elif self._state == 'HOOK':     self._do_hook()
        elif self._state == 'CLIMB':    self._do_climb()
        elif self._state == 'REACH':    self._do_reach()
        elif self._state == 'GRASP':    self._do_grasp()
        elif self._state == 'RETREAT':  self._do_retreat()
        elif self._state == 'DESCEND':  self._do_descend()
        elif self._state == 'DONE':     self._do_done()

    # ── State handlers ────────────────────────────────────────────────────────
    def _do_idle(self):
        self._stop()

    def _do_approach(self):
        """Drive forward toward shelf until detector sees the rail close."""
        if self._detector_status == 'SEARCHING':
            # Pan camera to find the rail
            self._send_joint({'camera_pan_joint': 0.0})
            self._drive(linear=0.15)
            return
        if self._rail_pose is None:
            return
        depth = self._rail_pose.pose.position.z
        if depth < 0.45:
            self._stop()
            self._state = 'ALIGN'
        else:
            # Steer toward rail centre
            lateral = self._rail_pose.pose.position.x
            angular = -1.5 * lateral  # proportional yaw correction
            angular = max(-0.8, min(0.8, angular))
            self._drive(linear=0.12, angular=angular)

    def _do_align(self):
        """Fine lateral alignment: robot inches sideways until rail is centred."""
        if self._detector_status == 'ALIGNED':
            self._stop()
            self._state = 'HOOK'
            return
        if self._rail_pose is None:
            return
        lateral = self._rail_pose.pose.position.x
        # Rotate in place to reduce lateral offset
        angular = -2.0 * lateral
        angular = max(-0.3, min(0.3, angular))
        self._drive(linear=0.0, angular=angular)

    def _do_hook(self):
        """Swing clamp_arm up to 45° (0.785 rad) to engage the rail."""
        target_angle = 0.785
        current = self._joint_pos.get('clamp_arm_joint', 0.0)
        if abs(current - target_angle) < 0.05:
            # Close jaw onto rail
            self._send_joint({'clamp_jaw_joint': 0.4})
            self._state = 'CLIMB'
        else:
            self._send_joint({'clamp_arm_joint': target_angle})

    def _do_climb(self):
        """Extend climb_joint to target shelf level height."""
        current = self._joint_pos.get('climb_joint', 0.0)
        target  = self._target_level
        if abs(current - target) < 0.03:
            self._state = 'REACH'
        else:
            self._send_joint({'climb_joint': target})

    def _do_reach(self):
        """Rotate manipulator arm to point at bottle on the shelf."""
        # Arm points forward-horizontal: shoulder at 0 rad (level), wrist at 0
        target_shoulder = -0.2   # slight downward for shelf surface approach
        current = self._joint_pos.get('manip_shoulder_joint', 0.0)
        if abs(current - target_shoulder) < 0.05:
            # Open gripper before approach
            self._send_joint({'gripper_finger_left_joint':  0.0,
                              'gripper_finger_right_joint': 0.0,
                              'gripper_joint': 0.0})
            self._state = 'GRASP'
        else:
            self._send_joint({'manip_shoulder_joint': target_shoulder})

    def _do_grasp(self):
        """Close gripper fingers around bottle (0.8 rad = fully closed)."""
        lf = self._joint_pos.get('gripper_finger_left_joint', 0.0)
        rf = self._joint_pos.get('gripper_finger_right_joint', 0.0)
        target_close = 0.75
        if lf > target_close - 0.05 and rf > target_close - 0.05:
            self.get_logger().info('Bottle grasped!')
            self._state = 'RETREAT'
        else:
            self._send_joint({'gripper_finger_left_joint':  target_close,
                              'gripper_finger_right_joint': target_close})

    def _do_retreat(self):
        """Pull arm back (shoulder up) so gripper clears the shelf edge."""
        target_shoulder = 0.5   # raise arm
        current = self._joint_pos.get('manip_shoulder_joint', 0.0)
        if abs(current - target_shoulder) < 0.05:
            self._state = 'DESCEND'
        else:
            self._send_joint({'manip_shoulder_joint': target_shoulder})

    def _do_descend(self):
        """Retract climb_joint back to 0 (ground level)."""
        current = self._joint_pos.get('climb_joint', 0.0)
        if abs(current) < 0.03:
            # Unhook jaw, lower arm
            self._send_joint({'clamp_arm_joint':  0.0,
                              'clamp_jaw_joint':  0.0,
                              'climb_joint':       0.0})
            self._state = 'DONE'
        else:
            self._send_joint({'climb_joint': 0.0})

    def _do_done(self):
        self._stop()
        self.get_logger().info('Climb-and-grab sequence complete. Returning to IDLE.')
        self._state = 'IDLE'

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _drive(self, linear: float = 0.0, angular: float = 0.0):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._vel_pub.publish(t)

    def _stop(self):
        self._drive(0.0, 0.0)

    def _send_joint(self, targets: dict):
        """Publish a JointState with desired positions."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(targets.keys())
        msg.position = list(targets.values())
        self._joint_pub.publish(msg)

    def _publish_state(self, s: str):
        msg = String()
        msg.data = s
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ClimbController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

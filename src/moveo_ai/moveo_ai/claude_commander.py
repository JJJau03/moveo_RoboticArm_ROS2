#!/usr/bin/env python3
"""ROS2 node for natural language control of the Moveo arm via the Anthropic Claude API."""

import json
import os
import sys
import threading

import anthropic
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

SYSTEM_PROMPT = """You are a controller for a BCN3D Moveo 5-DOF robotic arm running in ROS2 with MoveIt2.

## Robot Specifications

Planning group: "arm"
Joints (in order): joint1, joint2, joint3, joint4, joint5
All arm joint limits: -1.57 to +1.57 radians (approx. -90° to +90°)

Gripper group: "gripper"
Active joint: joint_R_gear  range 0.0 (open) to 1.57 (closed)

## Named Poses

Arm group:
  "home"  — all joints at 0.0 rad (arm pointing straight up, neutral position)
  "pose1" — joint1=-1.57, joint2=-1.57, joint3=-1.57, joint4=-1.57, joint5=1.57

Gripper group:
  "open"  — joint_R_gear = 0.0
  "close" — joint_R_gear = 1.57

## Response Format

Respond ONLY with a valid JSON object. No explanations, no markdown fences, no extra text.

Supported actions:

Move arm to a named pose:
  {"action": "move_to_pose", "pose": "<pose_name>"}

Move arm to specific joint angles in radians (values must be within ±1.57):
  {"action": "set_joints", "values": [j1, j2, j3, j4, j5]}

Open the gripper:
  {"action": "open_gripper"}

Close the gripper:
  {"action": "close_gripper"}

For ambiguous, unsafe, or out-of-range commands:
  {"action": "error", "message": "<brief reason>"}

Examples:
  "go home"                          → {"action": "move_to_pose", "pose": "home"}
  "open the gripper"                 → {"action": "open_gripper"}
  "move joints to 0, 0.5, -0.3, 0, 0" → {"action": "set_joints", "values": [0.0, 0.5, -0.3, 0.0, 0.0]}
  "move 500 meters forward"          → {"action": "error", "message": "position command out of range"}"""

_ARM_POSES = {
    'home':  {'joint1': 0.0,   'joint2': 0.0,   'joint3': 0.0,   'joint4': 0.0,   'joint5': 0.0},
    'pose1': {'joint1': -1.57, 'joint2': -1.57, 'joint3': -1.57, 'joint4': -1.57, 'joint5': 1.57},
}
_GRIPPER_POSES = {
    'open':  {'joint_R_gear': 0.0},
    'close': {'joint_R_gear': 1.57},
}
_ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def _joint_constraints(joint_values: dict, tol: float = 0.01) -> Constraints:
    c = Constraints()
    for name, value in joint_values.items():
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = float(value)
        jc.tolerance_above = tol
        jc.tolerance_below = tol
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    return c


class ClaudeCommander(Node):

    def __init__(self):
        super().__init__('claude_commander')

        self.declare_parameter('anthropic_api_key', '')
        api_key = self.get_parameter('anthropic_api_key').get_parameter_value().string_value
        if not api_key:
            api_key = os.environ.get('ANTHROPIC_API_KEY', '')
        if not api_key:
            self.get_logger().error(
                'ANTHROPIC_API_KEY not set. '
                'Export it as an environment variable or pass ~anthropic_api_key as a ROS2 param.'
            )
            raise RuntimeError('Missing ANTHROPIC_API_KEY')

        self._anthropic = anthropic.Anthropic(api_key=api_key)

        self._mg = ActionClient(self, MoveGroup, 'move_action')
        self.get_logger().info(
            'Waiting for move_group action server '
            '(start "ros2 launch moveo_bringup moveo.launch.py" if not running)...'
        )
        while not self._mg.wait_for_server(timeout_sec=5.0):
            self.get_logger().info('  ...still waiting for move_group...')
        self.get_logger().info('move_group connected.')

        self._sub = self.create_subscription(
            String, '/claude_command', self._topic_callback, 10
        )

        self.get_logger().info(
            'Claude Commander ready.\n'
            '  • Run commander_cli in another terminal to send commands\n'
            '  • Or publish to /claude_command (std_msgs/String)'
        )

    def _topic_callback(self, msg: String):
        threading.Thread(target=self._process, args=(msg.data,), daemon=True).start()

    def _process(self, text: str):
        text = text.strip()
        if not text:
            return

        self.get_logger().info(f'Command: "{text}"')

        try:
            response = self._anthropic.messages.create(
                model='claude-sonnet-4-6',
                max_tokens=256,
                system=SYSTEM_PROMPT,
                messages=[{'role': 'user', 'content': text}],
            )
            raw = response.content[0].text.strip()
        except anthropic.APIError as exc:
            self.get_logger().error(f'Anthropic API error: {exc}')
            return
        except Exception as exc:
            self.get_logger().error(f'Unexpected error calling Claude: {exc}')
            return

        self.get_logger().info(f'Claude → {raw}')

        try:
            cmd = json.loads(raw)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'JSON parse error: {exc}\nRaw: {raw}')
            return

        self._dispatch(cmd)

    def _dispatch(self, cmd: dict):
        action = cmd.get('action')

        if action == 'move_to_pose':
            pose = cmd.get('pose', '')
            if pose in _ARM_POSES:
                self._move('arm', _ARM_POSES[pose])
            elif pose in _GRIPPER_POSES:
                self._move('gripper', _GRIPPER_POSES[pose])
            else:
                self.get_logger().error(f'Unknown pose "{pose}". Known: {list(_ARM_POSES) + list(_GRIPPER_POSES)}')

        elif action == 'set_joints':
            values = cmd.get('values', [])
            if len(values) != 5:
                self.get_logger().error(f'set_joints: expected 5 values, got {len(values)}')
                return
            if any(abs(v) > 1.58 for v in values):
                self.get_logger().error(f'set_joints: value(s) exceed ±1.57 rad: {values}')
                return
            self._move('arm', dict(zip(_ARM_JOINTS, values)))

        elif action == 'open_gripper':
            self._move('gripper', _GRIPPER_POSES['open'])

        elif action == 'close_gripper':
            self._move('gripper', _GRIPPER_POSES['close'])

        elif action == 'error':
            self.get_logger().warn(f'Claude: {cmd.get("message", "unknown reason")}')

        else:
            self.get_logger().error(f'Unknown action: "{action}"')

    def _move(self, group: str, joint_values: dict):
        self.get_logger().info(f'Planning {group} → {joint_values}')

        goal = MoveGroup.Goal()
        goal.request.group_name = group
        goal.request.goal_constraints = [_joint_constraints(joint_values)]
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        done = threading.Event()
        outcome = [None]

        def _on_goal(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().error('move_group rejected the goal')
                outcome[0] = False
                done.set()
                return
            handle.get_result_async().add_done_callback(_on_result)

        def _on_result(future):
            val = future.result().result.error_code.val
            outcome[0] = (val == 1)
            if outcome[0]:
                self.get_logger().info(f'{group} motion complete.')
            else:
                self.get_logger().error(f'{group} motion failed (error_code={val})')
            done.set()

        self._mg.send_goal_async(goal).add_done_callback(_on_goal)
        done.wait(timeout=30.0)

        if outcome[0] is None:
            self.get_logger().error(f'{group} motion timed out after 30s')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ClaudeCommander()
    except RuntimeError:
        rclpy.shutdown()
        return

    if sys.stdin.isatty():
        def _stdin_loop():
            print('\nClaude Commander — interactive mode')
            print('Type a natural language command and press Enter.\n')
            while rclpy.ok():
                try:
                    text = input('Command> ')
                    if text.strip():
                        threading.Thread(target=node._process, args=(text,), daemon=True).start()
                except (EOFError, KeyboardInterrupt):
                    break

        threading.Thread(target=_stdin_loop, daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

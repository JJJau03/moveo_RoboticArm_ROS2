#!/usr/bin/env python3
"""Simple terminal prompt for sending natural language commands to the Moveo arm."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommanderCLI(Node):
    def __init__(self):
        super().__init__('commander_cli')
        self._pub = self.create_publisher(String, '/claude_command', 10)

    def send(self, text: str):
        msg = String()
        msg.data = text
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommanderCLI()

    print('─' * 50)
    print('  Moveo Arm — Natural Language Commander')
    print('─' * 50)
    print('  Type a command in plain English and press Enter.')
    print('  Examples:')
    print('    go home')
    print('    open the gripper')
    print('    move joints to 0, 0.5, -0.3, 0, 0')
    print('  Press Ctrl+C to quit.')
    print('─' * 50)

    try:
        while rclpy.ok():
            try:
                text = input('\nCommand> ').strip()
            except EOFError:
                break
            if text:
                node.send(text)
                print(f'  → Sent. Watch the arm terminal for Claude\'s response.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

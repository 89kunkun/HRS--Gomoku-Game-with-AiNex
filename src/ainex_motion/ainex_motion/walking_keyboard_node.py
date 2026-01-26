"""
walking_keyboard_node
---------------------
Keyboard teleoperation node for AiNex walking.

This node:
- reads keyboard input
- maps keys to walking commands
- uses WalkingController as backend
"""

import sys 
import termios
import tty

import rclpy

from ainex_motion.walking_controller import WalkingController

def get_key():
    """Read one key from terminal (Linux)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rclpy.init()

    wc = WalkingController(node_name="walking_keyboard_node")
    wc.activate()

    speed = 0.5
    turn = 0.5

    wc.get_logger().info(
        "Keyboard walking control:\n"
        "  w/s : forward/backward\n"
        "  a/d : strafe left/right\n"
        "  q/e : rotate left/right\n"
        "  space : stop\n"
        "  +/- : speed up/down\n"
        "  CTRL-C : quit"
    )

    try:
        while rclpy.ok():
            key = get_key()

            if key == 'w':
                wc.send_cmd(x=speed)
            elif key == 's':
                wc.send_cmd(x=-speed)
            elif key == 'a':
                wc.send_cmd(y=speed)
            elif key == 'd':
                wc.send_cmd(y = -speed)
            elif key == 'q':
                wc.send_cmd(yaw=turn)
            elif key == 'e':
                wc.send_cmd(yaw=-turn)
            elif key == ' ':
                wc.stop()
            elif key == '+':
                speed = min(speed * 1.1, 1.0)
                turn = min(turn * 1.1, 1.0)
                wc.get_logger().info(
                    f"Speed increased: speed = {speed:.2f}, turn={turn:.2f}"
                )
            elif key == '-':
                speed = max(speed * 0.9, 0.05)
                turn  = max(turn * 0.9, 0.05)
                wc.get_logger().info(
                    f"Speed decreased: speed={speed:.2f}, turn={turn:.2f}"
                )
            elif key == '\x03': # CTRL-C
                wc.stop()
                break
            else:
                wc.stop()
    
    finally:
        wc.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
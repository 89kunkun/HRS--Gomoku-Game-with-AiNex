#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class PublishBoardIJNode(Node):
    """
    发布一个棋盘格点 (i,j) = (2,1)
    约定：
      pose.position.x = i
      pose.position.y = j
      pose.position.z = 0
    """

    def __init__(self):
        super().__init__('publish_board_ij_node')

        # -------- params --------
        self.declare_parameter('topic', '/piece_ij')
        self.declare_parameter('board_frame', 'board_frame')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.topic = self.get_parameter('topic').value
        self.board_frame = self.get_parameter('board_frame').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        # -------- publisher --------
        self.pub = self.create_publisher(PoseStamped, self.topic, 10)

        # -------- timer --------
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.get_logger().info(
            f"Publishing board grid point (2,1)\n"
            f"  topic: {self.topic}\n"
            f"  frame: {self.board_frame}\n"
            f"  rate : {rate} Hz"
        )

    def on_timer(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.board_frame

        # 棋盘格点坐标 (i,j)
        msg.pose.position.x = 4.0
        msg.pose.position.y = 9.0
        msg.pose.position.z = 0.0

        # 朝向无意义，给单位四元数
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PublishBoardIJNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

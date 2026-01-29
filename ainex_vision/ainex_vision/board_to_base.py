#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point


class PieceIJToBasePoseNode(Node):
    """
    Subscribe:
      - piece_ij_topic (PoseStamped): pose.position.x=i, pose.position.y=j
    Compute in board_frame:
      x = i*grid_spacing, y = j*grid_spacing
    TF2:
      base_link <- board_frame
    Publish:
      - piece_point_base_topic (PointStamped, frame_id=base_frame)
    """

    def __init__(self):
        super().__init__('piece_ij_to_base_pose_node')

        # -------- params --------
        self.declare_parameter('piece_ij_topic', '/piece_ij')
        self.declare_parameter('board_frame', 'board_frame')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('grid_spacing_mm', 11.066667)
        self.declare_parameter('tf_timeout_sec', 0.05)

        self.declare_parameter('piece_point_base_topic', '/piece_point_base_from_ij')

        self.piece_ij_topic = self.get_parameter('piece_ij_topic').value
        self.board_frame = self.get_parameter('board_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.grid_spacing_m = float(self.get_parameter('grid_spacing_mm').value) / 1000.0
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)

        self.piece_point_base_topic = self.get_parameter('piece_point_base_topic').value

        # -------- tf2 --------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -------- pub/sub --------
        self.pub_base = self.create_publisher(
            PointStamped, self.piece_point_base_topic, 10
        )
        self.sub_ij = self.create_subscription(
            PoseStamped, self.piece_ij_topic, self.on_ij, 10
        )

        self.get_logger().info(
            "Started PieceIJToBasePoseNode (PoseStamped)\n"
            f"  sub: {self.piece_ij_topic} (PoseStamped: x=i, y=j)\n"
            f"  board_frame={self.board_frame}, base_frame={self.base_frame}\n"
            f"  grid_spacing={self.grid_spacing_m} m\n"
            f"  pub: {self.piece_point_base_topic} (PointStamped in {self.base_frame})\n"
        )

    def on_ij(self, msg: PoseStamped):
        # 1) lookup TF: base <- board
        try:
            tf_base_from_board = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.board_frame,
                rclpy.time.Time(),  # latest
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF lookup failed: {self.base_frame} <- {self.board_frame}: {e}"
            )
            return

        # 2) 读取棋盘格点 (i,j)
        i = float(msg.pose.position.x)
        j = float(msg.pose.position.y)

        # 3) 构造 board_frame 下的 PointStamped
        ps_board = PointStamped()
        ps_board.header.stamp = (
            msg.header.stamp
            if (msg.header.stamp.sec or msg.header.stamp.nanosec)
            else self.get_clock().now().to_msg()
        )
        ps_board.header.frame_id = self.board_frame

        ps_board.point.x = i * self.grid_spacing_m
        ps_board.point.y = j * self.grid_spacing_m
        ps_board.point.z = 0.0

        # 4) TF 变换到 base_frame
        ps_base = do_transform_point(ps_board, tf_base_from_board)
        ps_base.header.stamp = ps_board.header.stamp
        ps_base.header.frame_id = self.base_frame

        # 5) 发布
        self.pub_base.publish(ps_base)


def main():
    rclpy.init()
    node = PieceIJToBasePoseNode()
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

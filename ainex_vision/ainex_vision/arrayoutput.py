#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int16MultiArray, MultiArrayDimension

cavpath='/home/hrs2025/git_website/HRS--Gomoku-Game-with-AiNex/ainex_vision/ainex_vision/piece_grid.csv'

class PieceArrayOutput(Node):
    def __init__(self):
        super().__init__('piece_array_output')

        self.declare_parameter('piece_grid_topic', '/piece_nearest_grid')
        self.declare_parameter('remove_after_sec', 3.0)
        self.declare_parameter('min_hits_per_sec', 10)
        self.declare_parameter('min_duration_sec', 10.0)
        self.declare_parameter('array_topic', '/piece_grid_array')
        self.declare_parameter('output_file', cavpath)
        self.declare_parameter('write_header', True)

        self.piece_grid_topic = self.get_parameter('piece_grid_topic').value
        self.remove_after_sec = float(self.get_parameter('remove_after_sec').value)
        self.min_hits_per_sec = int(self.get_parameter('min_hits_per_sec').value)
        self.min_duration_sec = float(self.get_parameter('min_duration_sec').value)
        self.array_topic = self.get_parameter('array_topic').value
        self.output_file = self.get_parameter('output_file').value
        self.write_header = bool(self.get_parameter('write_header').value)

        self.sub = self.create_subscription(
            PointStamped, self.piece_grid_topic, self.on_piece_grid, 10
        )
        self.pub_array = self.create_publisher(Int16MultiArray, self.array_topic, 10)

        self.tracks = {}
        self.stable_indices = []
        self.last_seen_by_index = {}

        self._init_output_file()

        self.get_logger().info(
            f"Sub: {self.piece_grid_topic} | "
            f"Pub: {self.array_topic} | output_file={self.output_file}"
        )

    def on_piece_grid(self, msg: PointStamped):
        now = Time.from_msg(msg.header.stamp) if msg.header.stamp.sec != 0 else self.get_clock().now()
        i = int(round(msg.point.x))
        j = int(round(msg.point.y))
        if i < 0 or j < 0:
            return
        if i > 16 or j > 16:
            return
        ij = (i, j)

        track = self.tracks.get(ij)
        if track is None:
            track = {
                'first_seen': now,
                'last_seen': now,
                'stable': False,
                'hits': [],
            }
            self.tracks[ij] = track
        else:
            track['last_seen'] = now

        track['hits'].append(now)
        self._prune_hits(track, now)

        if not track['stable']:
            age_ns = (now - track['first_seen']).nanoseconds
            duration_ok = age_ns >= int(self.min_duration_sec * 1e9)
            hits_ok = len(track['hits']) >= self.min_hits_per_sec
            if duration_ok and hits_ok:
                track['stable'] = True
                if ij not in self.stable_indices:
                    self.stable_indices.append(ij)
                    self.last_seen_by_index[ij] = now
                    self._append_to_file(i, j)
                    self._publish_array()
                    self.get_logger().info(
                        f"Piece added: ({i},{j})"
                    )
        else:
            self.last_seen_by_index[ij] = now

        self._prune_tracks(now)
        self._prune_stale_indices(now)

    def _init_output_file(self):
        if not self.output_file:
            return
        out_dir = os.path.dirname(self.output_file)
        if out_dir:
            os.makedirs(out_dir, exist_ok=True)
        with open(self.output_file, 'w', encoding='utf-8') as f:
            if self.write_header:
                f.write("i,j\n")

    def _append_to_file(self, i: int, j: int):
        if not self.output_file:
            return
        with open(self.output_file, 'a', encoding='utf-8') as f:
            f.write(f"{i},{j}\n")

    def _publish_array(self):
        if not self.stable_indices:
            return
        msg = Int16MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(
                label='pieces',
                size=len(self.stable_indices),
                stride=len(self.stable_indices) * 3,
            ),
            MultiArrayDimension(
                label='attrs',
                size=3,
                stride=3,
            ),
        ]
        data = []
        for i, j in self.stable_indices:
            data.extend([int(i), int(j), 1])
        msg.data = data
        self.pub_array.publish(msg)

    def _prune_stale_indices(self, now: Time):
        remove_ns = int(self.remove_after_sec * 1e9)
        if remove_ns <= 0 or not self.stable_indices:
            return
        keep = []
        changed = False
        for ij in self.stable_indices:
            last = self.last_seen_by_index.get(ij)
            if last is None or (now - last).nanoseconds > remove_ns:
                changed = True
                continue
            keep.append(ij)
        if changed:
            self.stable_indices = keep
            self.last_seen_by_index = {ij: self.last_seen_by_index[ij] for ij in keep}
            self._rewrite_file()
            self._publish_array()

    def _rewrite_file(self):
        if not self.output_file:
            return
        with open(self.output_file, 'w', encoding='utf-8') as f:
            if self.write_header:
                f.write("i,j\n")
            for i, j in self.stable_indices:
                f.write(f"{i},{j}\n")

    def _prune_tracks(self, now: Time):
        timeout_ns = int(self.remove_after_sec * 1e9)
        if timeout_ns <= 0 or not self.tracks:
            return
        self.tracks = {
            ij: t for ij, t in self.tracks.items()
            if (now - t['last_seen']).nanoseconds <= timeout_ns
        }

    def _prune_hits(self, track, now: Time):
        window_ns = int(1e9)
        track['hits'] = [
            t for t in track['hits']
            if (now - t).nanoseconds <= window_ns
        ]


def main():
    rclpy.init()
    node = PieceArrayOutput()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

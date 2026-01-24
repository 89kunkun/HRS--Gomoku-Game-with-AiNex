from __future__ import annotations

import threading
from typing import Optional, Tuple, List

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Int16MultiArray

# Important: keep this as an intra-package import
from gomoku_visualization.ui.renderer import GomokuRenderer, VizState, MoveItem


BOARD_N = 15


def clamp_rc(r: int, c: int) -> Optional[Tuple[int, int]]:
    if 0 <= r < BOARD_N and 0 <= c < BOARD_N:
        return (r, c)
    return None


class GomokuVisualizerNode(Node):
    """
    Subscribe:
      /gomoku/move           Int16MultiArray  [row, col, player]  player: 1 black, 2 white
      /gomoku/current_player String          "black" / "white"
      /gomoku/warning        String
      /gomoku/reset          Empty

    Maintain local:
      board[15,15], move_history, last_move
    """

    def __init__(self):
        super().__init__("gomoku_visualizer")

        self._lock = threading.Lock()

        # local state cache
        self._board = np.zeros((BOARD_N, BOARD_N), dtype=np.int8)
        self._current_player = "black"
        self._last_move: Optional[Tuple[int, int]] = None
        self._warning = ""
        self._move_history: List[MoveItem] = []

        # renderer (pygame)
        self._renderer = GomokuRenderer(board_size=BOARD_N)

        # subscribers
        self.create_subscription(Int16MultiArray, "/gomoku/move", self.cb_move, 10)
        self.create_subscription(String, "/gomoku/current_player", self.cb_current_player, 10)
        self.create_subscription(String, "/gomoku/warning", self.cb_warning, 10)
        self.create_subscription(Empty, "/gomoku/reset", self.cb_reset, 10)

        # timer: drive GUI refresh (30 FPS)
        self.create_timer(1.0 / 30.0, self.on_timer)

        self.get_logger().info("GomokuVisualizerNode started (ROS2).")

    # ---------- callbacks ----------
    def cb_current_player(self, msg: String):
        p = msg.data.strip().lower()
        if p not in ("black", "white"):
            with self._lock:
                self._warning = f"current_player invalid: {msg.data}"
            return
        with self._lock:
            self._current_player = p

    def cb_warning(self, msg: String):
        with self._lock:
            self._warning = msg.data.strip()

    def cb_reset(self, msg: Empty):
        with self._lock:
            self._board[:] = 0
            self._move_history.clear()
            self._last_move = None
            self._warning = ""
        self.get_logger().info("Board reset received.")

    def cb_move(self, msg: Int16MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            with self._lock:
                self._warning = "move msg invalid: need [row,col,player]"
            return

        r, c, player = int(data[0]), int(data[1]), int(data[2])
        rc = clamp_rc(r, c)
        if rc is None:
            with self._lock:
                self._warning = f"move out of range: ({r},{c})"
            return

        if player not in (1, 2):
            with self._lock:
                self._warning = f"player invalid: {player} (expect 1=black,2=white)"
            return

        with self._lock:
            if int(self._board[r, c]) != 0:
                self._warning = f"cell occupied: ({r},{c})"
                return

            # update board
            self._board[r, c] = player
            self._last_move = (r, c)

            # history: player -> string
            pstr = "black" if player == 1 else "white"
            self._move_history.append(
                MoveItem(move_idx=len(self._move_history) + 1, player=pstr, row=r, col=c)
            )

            # clear warning on valid move (optional)
            self._warning = ""

    # ---------- render loop ----------
    def on_timer(self):
        # allow closing pygame window to stop node
        if not self._renderer.running:
            self.get_logger().info("Pygame window closed. Shutting down node.")
            rclpy.shutdown()
            return

        with self._lock:
            snap = VizState(
                board=self._board.copy(),
                current_player=self._current_player,
                last_move=self._last_move,
                warning=self._warning,
                move_history=list(self._move_history),
            )

        self._renderer.render(snap, fps=30)


def main():
    rclpy.init()
    node = GomokuVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

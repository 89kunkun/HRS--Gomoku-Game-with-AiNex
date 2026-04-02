# src/ui/renderer.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Dict
import time

import pygame
import numpy as np

from gomoku_visualization.ui.theme import THEME


@dataclass
class MoveItem:
    move_idx: int
    player: str  # "black" / "white"
    row: int
    col: int
    ts: float = field(default_factory=time.time)


@dataclass
class VizState:
    board: np.ndarray  # (15,15) int
    current_player: str = "black"
    last_move: Optional[Tuple[int, int]] = None
    #detected_move: Optional[Tuple[int, int]] = None
    warning: str = ""
    move_history: List[MoveItem] = field(default_factory=list)

    # for light animation
    _last_move_anim_start: float = 0.0
    _last_move_anim_pos: Optional[Tuple[int, int]] = None


class GomokuRenderer:
    """
    A polished 2D renderer: board + side panel (move history + status).
    """

    def __init__(self, board_size: int = 15, cell: int = 40, margin: int = 40, history_width: int = 320):
        self.n = board_size
        self.cell = cell
        self.margin = margin
        self.board_px = (self.n - 1) * self.cell + 2 * self.margin
        self.history_width = history_width
        self.bottom_bar_h = 72

        self.win_w = self.board_px + self.history_width
        self.win_h = self.board_px + self.bottom_bar_h

        pygame.init()
        pygame.display.set_caption("Ainex Gomoku - Real-time Visualization")
        self.screen = pygame.display.set_mode((self.win_w, self.win_h))
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.SysFont("Arial", 22, bold=True)
        self.font = pygame.font.SysFont("Arial", 18)
        self.font_small = pygame.font.SysFont("Arial", 16)

        self.running = True

    # ---------- coordinate helpers ----------
    def rc_to_xy(self, row: int, col: int) -> Tuple[int, int]:
        """
        Convert board row/col (origin at bottom-left, row=0 is bottom) to screen x,y.
        y 轴向上增加，x 轴向右增加。
        """
        x = self.margin + col * self.cell
        # flip row so屏幕原点仍在左上，视觉上 (0,0) 在左下角
        y = self.margin + (self.n - 1 - row) * self.cell
        return x, y

    # ---------- draw primitives ----------
    def _draw_shadowed_circle(self, center: Tuple[int, int], radius: int, color: Tuple[int, int, int], edge: bool):
        # soft shadow
        shadow = (0, 0, 0)
        shadow_offset = (2, 3)
        pygame.draw.circle(self.screen, shadow, (center[0] + shadow_offset[0], center[1] + shadow_offset[1]), radius)

        pygame.draw.circle(self.screen, color, center, radius)
        if edge:
            pygame.draw.circle(self.screen, THEME["stone_edge"], center, radius, 2)

        # highlight spot (gives 3D feel)
        hx, hy = center[0] - radius // 3, center[1] - radius // 3
        pygame.draw.circle(self.screen, (255, 255, 255), (hx, hy), max(3, radius // 6))

    def _draw_text(self, text: str, pos: Tuple[int, int], color=(0, 0, 0), font=None):
        if font is None:
            font = self.font
        surf = font.render(text, True, color)
        self.screen.blit(surf, pos)

    def _draw_panel(self, state: VizState):
        panel_x = self.board_px
        panel_y = 0
        panel_w = self.history_width
        panel_h = self.board_px

        # background
        pygame.draw.rect(self.screen, THEME["panel_bg"], (panel_x, panel_y, panel_w, panel_h))
        pygame.draw.rect(self.screen, THEME["panel_border"], (panel_x, panel_y, panel_w, panel_h), 1)

        # title
        self._draw_text("Move History", (panel_x + 16, 14), THEME["text"], self.font_title)

        # list area
        list_top = 50
        line_h = 26
        max_lines = (panel_h - list_top - 16) // line_h

        history = state.move_history[-max_lines:]
        # highlight latest in list
        latest_key = None
        if state.last_move is not None and len(state.move_history) > 0:
            latest_key = (state.move_history[-1].row, state.move_history[-1].col, state.move_history[-1].player)

        y = list_top
        base_idx = max(0, len(state.move_history) - len(history))

        for i, item in enumerate(history):
            is_latest = latest_key == (item.row, item.col, item.player)
            if is_latest:
                pygame.draw.rect(
                    self.screen,
                    THEME["history_highlight_bg"],
                    (panel_x + 10, y - 2, panel_w - 20, line_h),
                    border_radius=6,
                )

            # text
            p = "Black" if item.player == "black" else "White"
            color = (0, 0, 0) if item.player == "black" else THEME["muted_text"]
            # 显示为 (x,y)，其中 x=col(水平), y=row(垂直)
            text = f"{base_idx + i + 1:>3}. {p:<5} ({item.col:>2},{item.row:>2})"
            self._draw_text(text, (panel_x + 16, y), color, self.font)
            y += line_h

    def _draw_bottom_bar(self, state: VizState):
        bar_y = self.board_px
        pygame.draw.rect(self.screen, THEME["window_bg"], (0, bar_y, self.win_w, self.bottom_bar_h))
        pygame.draw.line(self.screen, THEME["panel_border"], (0, bar_y), (self.win_w, bar_y), 1)

        # current player
        p = "Black" if state.current_player == "black" else "White"
        self._draw_text(f"Turn: {p}", (18, bar_y + 12), THEME["text"], self.font_title)

        # last move 
        if state.last_move:
            r, c = state.last_move
            lm = f"(x={c}, y={r})"
        else:
            lm = "None"
        #dm = f"{state.detected_move}" if state.detected_move else "None"
        self._draw_text(f"Last move: {lm}", (18, bar_y + 40), THEME["text"], self.font_small)
        #self._draw_text(f"Detected: {dm}", (260, bar_y + 40), THEME["text"], self.font_small)

        # warning
        if state.warning:
            self._draw_text(f"Warning: {state.warning}", (520, bar_y + 22), THEME["warning"], self.font)

    def _draw_grid(self):
        # board background
        pygame.draw.rect(self.screen, THEME["board_bg"], (0, 0, self.board_px, self.board_px))

        # grid lines
        for i in range(self.n):
            x0, y0 = self.rc_to_xy(0, i)
            x1, y1 = self.rc_to_xy(self.n - 1, i)
            pygame.draw.line(self.screen, THEME["grid"], (x0, y0), (x1, y1), 2)

        for i in range(self.n):
            x0, y0 = self.rc_to_xy(i, 0)
            x1, y1 = self.rc_to_xy(i, self.n - 1)
            pygame.draw.line(self.screen, THEME["grid"], (x0, y0), (x1, y1), 2)

        # star points (nice touch)
        star_points = [(3, 3), (3, 11), (11, 3), (11, 11), (7, 7)]
        for r, c in star_points:
            x, y = self.rc_to_xy(r, c)
            pygame.draw.circle(self.screen, THEME["grid"], (x, y), 4)

    def _draw_stones(self, board: np.ndarray):
        radius = self.cell // 2 - 4
        for r in range(self.n):
            for c in range(self.n):
                v = int(board[r, c])
                if v == 0:
                    continue
                center = self.rc_to_xy(r, c)
                if v == 1:
                    self._draw_shadowed_circle(center, radius, THEME["black_stone"], edge=False)
                elif v == 2:
                    self._draw_shadowed_circle(center, radius, THEME["white_stone"], edge=True)

    def _draw_last_move_highlight(self, state: VizState):
        if not state.last_move:
            return
        r, c = state.last_move
        center = self.rc_to_xy(r, c)

        # subtle "breathing" highlight
        t = time.time()
        pulse = (np.sin(t * 4.0) + 1.0) * 0.5  # 0..1
        radius = int((self.cell // 2) + 6 + pulse * 4)

        pygame.draw.circle(self.screen, THEME["highlight"], center, radius, 3)

    
    # ---------- public render loop ----------
    def pump_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

    def render(self, state: VizState, fps: int = 30):
        self.pump_events()

        self.screen.fill(THEME["window_bg"])

        self._draw_grid()
        self._draw_stones(state.board)
        self._draw_last_move_highlight(state)
        

        self._draw_panel(state)
        self._draw_bottom_bar(state)

        pygame.display.flip()
        self.clock.tick(fps)

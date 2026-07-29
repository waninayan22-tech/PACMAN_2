"""
╔══════════════════════════════════════════════════════════════════════════════╗
║  PacBot – Autonomous Maze Simulation Game                                    ║
║  ──────────────────────────────────────────────────────────────────────────  ║
║  A robotics challenge where an autonomous PacBot navigates a maze,           ║
║  collects Pallets, avoids Ghost bots, and escapes through the Exit.          ║
║                                                                              ║
║  QUICK START:                                                                ║
║    pip install pygame                                                        ║
║    python pacman.py                                                          ║
║                                                                              ║
║  CONTROLS:                                                                   ║
║    Arrow Keys / WASD  → Move PacBot (Manual Mode)                           ║
║    TAB                → Toggle Manual / Autonomous Mode                      ║
║    R                  → Restart                                              ║
║    ESC                → Quit                                                 ║
║                                                                              ║
║  ALGORITHMS:                                                                 ║
║    A* Pathfinding with ghost-danger cost weighting                           ║
║    DecisionEngine state machine for autonomous behaviour                     ║
║    Simulated VL53L1X ToF sensor raycasting                                   ║
║                                                                              ║
║  TARGET HARDWARE:                                                            ║
║    ESP32 WROOM-32  → Main Controller                                         ║
║    VL53L1X ×4     → ToF Wall / Distance Sensors                             ║
║    TB6612FNG      → Dual Motor Driver                                        ║
║    N20 + Encoder  → Drive Motors                                             ║
║    N20 Wheels     → Physical Motion                                          ║
╚══════════════════════════════════════════════════════════════════════════════╝
"""

import pygame
import sys
import math
import heapq
from enum import Enum, auto
from typing import List, Tuple, Optional, Dict

# ══════════════════════════════════════════════════════════════════════════════
#  CONSTANTS
# ══════════════════════════════════════════════════════════════════════════════

TILE          = 30          # pixels per grid tile
COLS          = 25          # maze columns
ROWS          = 19          # maze rows
MAZE_W        = COLS * TILE  # 750
MAZE_H        = ROWS * TILE  # 570
PANEL_W       = 320
TOP_H         = 62
BOT_H         = 46
WIN_W         = MAZE_W + PANEL_W       # 1070
WIN_H         = TOP_H + MAZE_H + BOT_H # 678

FPS           = 60
GAME_TIME     = 90          # seconds
DANGER_RADIUS = 5           # tiles – ghost danger zone radius
CRITICAL_TIME = 20          # seconds left → force escape
GHOST_SPEED   = 60.0        # px/s
PACBOT_SPEED  = 80.0        # px/s
TILE_CM       = 30          # 1 tile = 30 cm (sensor display scale)

WALL  = 0
FLOOR = 1

# ── Colours ───────────────────────────────────────────────────────────────────
C_BG         = (  5,   5,  18)
C_WALL       = ( 20,  50, 180)
C_WALL_EDGE  = ( 60, 100, 255)
C_FLOOR      = (  8,   8,  25)
C_PALLET     = (255, 255, 200)
C_EXIT       = (  0, 255, 130)
C_EXIT_DARK  = (  0, 140,  75)
C_START      = (100, 200, 255)
C_PACBOT     = (255, 220,   0)
C_PACBOT_EYE = ( 30,  30,  30)
GHOST_COLORS = {
    'Blinky': (255,  55,  55),
    'Pinky':  (255, 160, 210),
    'Inky':   (  0, 220, 230),
    'Clyde':  (255, 165,  50),
}
C_BLACK      = (  0,   0,   0)
C_WHITE      = (255, 255, 255)
C_TOP_BG     = (  4,   4,  14)
C_PANEL_BG   = ( 10,  10,  28)
C_PANEL_SEP  = ( 25,  35,  80)
C_ACCENT     = (  0, 200, 255)
C_GREEN      = (  0, 220, 120)
C_RED        = (255,  70,  70)
C_ORANGE     = (255, 165,  50)
C_YELLOW     = (255, 220,  50)
C_GRAY       = (120, 120, 145)
C_DGRAY      = ( 40,  40,  60)
C_PATH       = ( 80, 180, 255)
C_DANGER     = (255,  40,  40)

# ══════════════════════════════════════════════════════════════════════════════
#  MAZE LAYOUT  (25 cols × 19 rows)
#  '#'=wall  '.'=pallet floor  'E'=exit(floor)
# ══════════════════════════════════════════════════════════════════════════════

MAZE_LAYOUT = [
    "#########################",   # row  0
    "#.......................E#",   # row  1  ← full corridor; Exit col 23
    "#.###.#####.###.#####.#.#",   # row  2
    "#.#...#.....#...#.....#.#",   # row  3
    "#.#.###.###.#.###.###.#.#",   # row  4
    "#.#...#.#...#...#.#...#.#",   # row  5
    "#.#####.#.#####.#.#####.#",   # row  6
    "#.......#.......#.......#",   # row  7  ← open corridor
    "#.#.###.#.#####.#.###.#.#",   # row  8
    "#.#.#...#.#...#.#...#.#.#",   # row  9
    "#.#.#.###.#.#.#.###.#.#.#",   # row 10
    "#...#.....#...#.....#...#",   # row 11
    "#.#.#.###.#.#.#.###.#.#.#",   # row 12
    "#.#.#...#.#...#.#...#.#.#",   # row 13
    "#.#.###.#.#####.#.###.#.#",   # row 14
    "#.......#.......#.......#",   # row 15  ← open corridor
    "#.#####.#.#####.#.#####.#",   # row 16
    "#.#...#.#...#...#.#...#.#",   # row 17
    "#########################",   # row 18
]

PACBOT_START = (1, 11)   # (col, row)
EXIT_POS     = (23, 1)   # (col, row) – where 'E' is

# Ghost patrol waypoints – ghosts use A* to navigate between them
GHOST_ROUTES = {
    'Blinky': [(2, 1),  (21, 1)],                   # Top corridor L↔R
    'Pinky':  [(1, 2),  (1, 16)],                    # Left column  T↔B
    'Inky':   [(23, 2), (23, 16)],                   # Right column T↔B
    'Clyde':  [(11, 1), (11, 11), (1, 11), (1, 16)], # Centre roam
}

# ══════════════════════════════════════════════════════════════════════════════
#  ENUMERATIONS
# ══════════════════════════════════════════════════════════════════════════════

class GameState(Enum):
    TITLE   = auto()
    PLAYING = auto()
    WIN     = auto()
    LOSE    = auto()

class GameMode(Enum):
    MANUAL     = auto()
    AUTONOMOUS = auto()

class AIDecision(Enum):
    IDLE              = "Idle"
    COLLECTING_PALLET = "Collecting Pallet"
    AVOIDING_GHOST    = "Ghost Threat! Evading"
    REROUTING         = "Rerouting..."
    ESCAPING          = "Escaping to Exit"
    STUCK             = "Stuck - Recalculating"

# ══════════════════════════════════════════════════════════════════════════════
#  MAZE
# ══════════════════════════════════════════════════════════════════════════════

class Maze:
    """Grid-based maze parsed from MAZE_LAYOUT strings."""

    def __init__(self):
        self.grid: List[List[int]] = []
        self._parse()
        self.wall_rects: List[pygame.Rect] = []
        self._build_rects()

    def _parse(self):
        for row_str in MAZE_LAYOUT:
            self.grid.append([WALL if ch == '#' else FLOOR for ch in row_str])

    def _build_rects(self):
        for r in range(ROWS):
            for c in range(COLS):
                if self.grid[r][c] == WALL:
                    self.wall_rects.append(pygame.Rect(c*TILE, r*TILE, TILE, TILE))

    def is_wall(self, col: int, row: int) -> bool:
        if col < 0 or col >= COLS or row < 0 or row >= ROWS:
            return True
        return self.grid[row][col] == WALL

    def get_neighbors(self, col: int, row: int) -> List[Tuple[int,int]]:
        nb = []
        for dc, dr in ((0,-1),(0,1),(-1,0),(1,0)):
            nc, nr = col+dc, row+dr
            if not self.is_wall(nc, nr):
                nb.append((nc, nr))
        return nb

    def draw(self, surf: pygame.Surface, ox: int, oy: int,
             font_sm: pygame.font.Font):
        surf.fill(C_FLOOR, (ox, oy, MAZE_W, MAZE_H))
        for rect in self.wall_rects:
            r = rect.move(ox, oy)
            pygame.draw.rect(surf, C_WALL, r)
            pygame.draw.line(surf, C_WALL_EDGE, r.topleft, r.topright, 1)
            pygame.draw.line(surf, C_WALL_EDGE, r.topleft, r.bottomleft, 1)
        # Exit tile
        ec, er = EXIT_POS
        exr = pygame.Rect(ox+ec*TILE+2, oy+er*TILE+2, TILE-4, TILE-4)
        pygame.draw.rect(surf, C_EXIT_DARK, exr, border_radius=4)
        pygame.draw.rect(surf, C_EXIT, exr, 2, border_radius=4)
        lbl = font_sm.render("EXIT", True, C_EXIT)
        surf.blit(lbl, (ox+ec*TILE+2, oy+er*TILE+8))
        # Start dot
        sc, sr = PACBOT_START
        pygame.draw.circle(surf, C_START,
                           (ox+sc*TILE+TILE//2, oy+sr*TILE+TILE//2), 4, 1)

# ══════════════════════════════════════════════════════════════════════════════
#  PALLET
# ══════════════════════════════════════════════════════════════════════════════

class Pallet:
    """Small collectible dot on the maze floor."""

    R = 3

    def __init__(self, col: int, row: int):
        self.col = col
        self.row = row
        self.collected = False

    @property
    def grid_pos(self) -> Tuple[int,int]:
        return (self.col, self.row)

    def draw(self, surf: pygame.Surface, ox: int, oy: int):
        if self.collected:
            return
        cx = ox + self.col*TILE + TILE//2
        cy = oy + self.row*TILE + TILE//2
        pygame.draw.circle(surf, C_PALLET, (cx, cy), self.R)
        pygame.draw.circle(surf, (180,180,130), (cx, cy), self.R+1, 1)

# ══════════════════════════════════════════════════════════════════════════════
#  A* PATHFINDING
# ══════════════════════════════════════════════════════════════════════════════

class Pathfinding:
    """
    A* search with optional ghost-danger cost weighting.

    f(n) = g(n) + h(n)
      g(n) = actual cost from start, including ghost-danger penalties
      h(n) = Manhattan distance heuristic (admissible for unweighted grids)

    Ghost penalty:  danger(cell) = Σ 50·exp(−dist_to_ghost / 2)
    This makes A* prefer ghost-free routes while still guaranteeing
    a path is found (the heuristic becomes inadmissible with penalties,
    relaxing optimality in favour of safety).
    """

    @staticmethod
    def manhattan(a: Tuple[int,int], b: Tuple[int,int]) -> int:
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    @staticmethod
    def ghost_penalty(col: int, row: int, ghosts: list,
                      radius: int = DANGER_RADIUS) -> float:
        total = 0.0
        for g in ghosts:
            gc, gr = g.grid_pos
            d = abs(col-gc) + abs(row-gr)
            if d < radius:
                total += 50.0 * math.exp(-d / 2.0)
        return total

    @staticmethod
    def astar(maze: 'Maze',
              start: Tuple[int,int],
              goal:  Tuple[int,int],
              ghosts: list = [],
              avoid: bool = True) -> List[Tuple[int,int]]:
        """
        Return list of (col,row) from the step after start up to goal.
        Returns [] if start==goal or no path found.
        """
        if start == goal:
            return []
        open_heap: List[Tuple] = []
        heapq.heappush(open_heap, (0.0, 0.0, start))
        came_from: Dict[Tuple,Tuple] = {}
        g_score:   Dict[Tuple,float] = {start: 0.0}

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)
            if cur == goal:
                path, node = [], cur
                while node in came_from:
                    path.append(node); node = came_from[node]
                path.reverse()
                return path
            if g > g_score.get(cur, float('inf')):
                continue
            for nb in maze.get_neighbors(cur[0], cur[1]):
                step = 1.0
                if avoid and ghosts:
                    step += Pathfinding.ghost_penalty(nb[0], nb[1], ghosts)
                ng = g + step
                if ng < g_score.get(nb, float('inf')):
                    came_from[nb] = cur
                    g_score[nb]   = ng
                    h = Pathfinding.manhattan(nb, goal)
                    heapq.heappush(open_heap, (ng+h, ng, nb))
        return []

# ══════════════════════════════════════════════════════════════════════════════
#  GHOST
# ══════════════════════════════════════════════════════════════════════════════

class Ghost:
    """
    Patrolling enemy bot.
    Uses A* to navigate a pre-set waypoint loop deterministically.
    Creates ghost-danger zones that PacBot's A* penalises.
    """

    R = TILE//2 - 2

    def __init__(self, name: str, color: Tuple,
                 waypoints: List[Tuple[int,int]], maze: 'Maze'):
        self.name      = name
        self.color     = color
        self.waypoints = waypoints
        self.maze      = maze
        start          = waypoints[0]
        self.px        = float(start[0]*TILE + TILE//2)
        self.py        = float(start[1]*TILE + TILE//2)
        self.wp_idx    = 1
        self.wp_dir    = 1
        self.nav_path: List[Tuple[int,int]] = []
        self.anim_t    = 0.0
        self.dir_x     = 1
        self.dir_y     = 0
        self._next_wp()

    def _next_wp(self):
        """A*-navigate to the next waypoint."""
        cur  = (int(self.px//TILE), int(self.py//TILE))
        goal = self.waypoints[self.wp_idx]
        self.nav_path = Pathfinding.astar(self.maze, cur, goal, avoid=False)

    @property
    def grid_pos(self) -> Tuple[int,int]:
        return (int(self.px//TILE), int(self.py//TILE))

    def update(self, dt: float):
        self.anim_t += dt
        if not self.nav_path:
            # Reached waypoint → advance patrol index (ping-pong)
            self.wp_idx += self.wp_dir
            if self.wp_idx >= len(self.waypoints):
                self.wp_dir = -1; self.wp_idx = len(self.waypoints)-2
            elif self.wp_idx < 0:
                self.wp_dir =  1; self.wp_idx = 1
            self._next_wp()
            return
        nc, nr = self.nav_path[0]
        tx = nc*TILE + TILE*0.5;  ty = nr*TILE + TILE*0.5
        dx = tx-self.px;           dy = ty-self.py
        dist = math.hypot(dx, dy)
        if dist < 1.5:
            self.px, self.py = tx, ty
            self.nav_path.pop(0)
        else:
            s = GHOST_SPEED*dt
            self.px += dx/dist*s;  self.py += dy/dist*s
            if abs(dx) >= abs(dy):
                self.dir_x = 1 if dx>0 else -1; self.dir_y = 0
            else:
                self.dir_x = 0; self.dir_y = 1 if dy>0 else -1

    def draw(self, surf: pygame.Surface, ox: int, oy: int):
        x = int(ox+self.px);  y = int(oy+self.py);  r = self.R;  c = self.color
        # Head
        pygame.draw.circle(surf, c, (x, y), r)
        # Body
        pygame.draw.rect(surf, c, pygame.Rect(x-r, y, r*2, r))
        # Wavy skirt
        wy = y+r+int(math.sin(self.anim_t*6)*2)
        bw = r*2//3
        for i in range(3):
            bx = x-r+i*bw+bw//2
            pygame.draw.circle(surf, C_FLOOR, (bx, wy), r//3)
        # Eyes
        for ex_off in (-r//3, r//3):
            ex = x+ex_off;  ey = y-r//3
            pygame.draw.circle(surf, C_WHITE, (ex, ey), r//3)
            pygame.draw.circle(surf, (20,20,180),
                               (ex+self.dir_x*2, ey+self.dir_y*2), r//5)

    def draw_danger_zone(self, surf: pygame.Surface, ox: int, oy: int):
        zone = pygame.Surface((MAZE_W, MAZE_H), pygame.SRCALPHA)
        cx = int(self.px);  cy = int(self.py)
        max_r = DANGER_RADIUS*TILE
        for ring in range(3, 0, -1):
            rad = max_r*ring//3
            alpha = 12 - ring*3
            if alpha > 0:
                pygame.draw.circle(zone, (*C_DANGER, alpha), (cx, cy), rad)
        surf.blit(zone, (ox, oy))

# ══════════════════════════════════════════════════════════════════════════════
#  PACBOT
# ══════════════════════════════════════════════════════════════════════════════

class PacBot:
    """
    Player / AI controlled robot.
    Manual Mode  : Arrow Keys / WASD
    Autonomous   : Follows A* path from DecisionEngine
    Rendered as an animated yellow arc (Pac-Man style).
    """

    R = TILE//2 - 2

    def __init__(self, col: int, row: int):
        self.col    = col;  self.row    = row
        self.px     = float(col*TILE + TILE//2)
        self.py     = float(row*TILE + TILE//2)
        self.score  = 0;  self.pallets = 0
        self.dir_x  = 1;  self.dir_y   = 0
        self.next_dx = 0; self.next_dy = 0
        self.path:   List[Tuple[int,int]] = []
        self.mouth   = 35.0
        self.mouth_open = True
        self.decision = AIDecision.IDLE

    @property
    def grid_pos(self) -> Tuple[int,int]:
        return (int(self.px//TILE), int(self.py//TILE))

    # ── Manual mode ───────────────────────────────────────────────────────────
    def handle_input(self, keys, maze: 'Maze'):
        if keys[pygame.K_LEFT]  or keys[pygame.K_a]: self.next_dx,self.next_dy=-1,0
        if keys[pygame.K_RIGHT] or keys[pygame.K_d]: self.next_dx,self.next_dy= 1,0
        if keys[pygame.K_UP]    or keys[pygame.K_w]: self.next_dx,self.next_dy= 0,-1
        if keys[pygame.K_DOWN]  or keys[pygame.K_s]: self.next_dx,self.next_dy= 0, 1

    def _try_step(self, dx: int, dy: int, maze: 'Maze') -> bool:
        nc = int(self.px//TILE)+dx;  nr = int(self.py//TILE)+dy
        if not maze.is_wall(nc, nr):
            self.px = nc*TILE+TILE*0.5;  self.py = nr*TILE+TILE*0.5
            if dx: self.dir_x,self.dir_y = dx,0
            if dy: self.dir_x,self.dir_y = 0,dy
            return True
        return False

    def update_manual(self, dt: float, maze: 'Maze'):
        self._anim_mouth(dt)
        if self.next_dx or self.next_dy:
            if self._try_step(self.next_dx, self.next_dy, maze):
                self.next_dx = self.next_dy = 0

    # ── Autonomous mode ───────────────────────────────────────────────────────
    def update_autonomous(self, dt: float, maze: 'Maze',
                          path: List[Tuple[int,int]]):
        """Smooth movement along planned A* path; pops consumed nodes."""
        self._anim_mouth(dt)
        if not path:
            return
        nc, nr = path[0]
        tx = nc*TILE+TILE*0.5;  ty = nr*TILE+TILE*0.5
        dx = tx-self.px;        dy = ty-self.py
        dist = math.hypot(dx, dy)
        if dist < 1.5:
            self.px,self.py = tx,ty
            path.pop(0)
        else:
            s = PACBOT_SPEED*dt
            self.px += dx/dist*s;  self.py += dy/dist*s
            if abs(dx) >= abs(dy): self.dir_x,self.dir_y=(1 if dx>0 else -1),0
            else:                  self.dir_x,self.dir_y=0,(1 if dy>0 else -1)

    # ── Animation ─────────────────────────────────────────────────────────────
    def _anim_mouth(self, dt: float):
        spd = 200.0
        if self.mouth_open:
            self.mouth -= spd*dt
            if self.mouth <= 4:   self.mouth_open = False
        else:
            self.mouth += spd*dt
            if self.mouth >= 38:  self.mouth_open = True

    # ── Rendering ─────────────────────────────────────────────────────────────
    def draw(self, surf: pygame.Surface, ox: int, oy: int):
        x = int(ox+self.px);  y = int(oy+self.py);  r = self.R
        facing = {(1,0):0.0,(-1,0):math.pi,(0,-1):-math.pi/2,(0,1):math.pi/2
                  }.get((self.dir_x,self.dir_y), 0.0)
        hm = math.radians(self.mouth)
        # Yellow body
        pygame.draw.circle(surf, C_PACBOT, (x,y), r)
        # Mouth cutout
        if self.mouth > 2:
            pts = [(x,y)]
            for i in range(9):
                a = (facing-hm) + 2*hm*i/8
                pts.append((x+math.cos(a)*(r+1), y+math.sin(a)*(r+1)))
            if len(pts) >= 3:
                pygame.draw.polygon(surf, C_FLOOR, pts)
        # Outline
        pygame.draw.circle(surf, (180,150,0), (x,y), r, 1)
        # Eye
        ea = facing - math.pi*0.35
        pygame.draw.circle(surf, C_PACBOT_EYE,
                           (int(x+math.cos(ea)*r*0.55),
                            int(y+math.sin(ea)*r*0.55)), 2)
        # Path overlay
        for i,(pc,pr) in enumerate(self.path[:15]):
            pygame.draw.circle(surf, C_PATH,
                               (ox+pc*TILE+TILE//2, oy+pr*TILE+TILE//2),
                               max(1, 3-i//5))

# ══════════════════════════════════════════════════════════════════════════════
#  DECISION ENGINE
# ══════════════════════════════════════════════════════════════════════════════

class DecisionEngine:
    """
    Autonomous decision-making system.

    Priority order (highest first):
      1. Ghost within CRIT_DIST tiles → AVOIDING_GHOST  (emergency flee)
      2. Time < CRITICAL_TIME         → ESCAPING        (head for exit)
      3. Safe pallet exists           → COLLECTING_PALLET
      4. No pallets left / reachable  → ESCAPING

    Paths are recomputed every ~30 frames or when the current path empties.
    Returns (AIDecision, new_path | None).  None means "keep current path."
    """

    CRIT_DIST = 3

    def __init__(self):
        self.state     = AIDecision.IDLE
        self.target    = None
        self.countdown = 0

    def update(self,
               pacbot:   'PacBot',
               maze:     'Maze',
               ghosts:   list,
               pallets:  list,
               time_rem: float,
               cur_path: list) -> Tuple[AIDecision, Optional[list]]:

        pos  = pacbot.grid_pos
        gdist = self._nearest_ghost(pos, ghosts)

        # ── 1. Emergency evasion ──────────────────────────────────────────────
        if gdist <= self.CRIT_DIST:
            self.state = AIDecision.AVOIDING_GHOST
            path = (Pathfinding.astar(maze, pos, EXIT_POS, ghosts, avoid=False) or
                    Pathfinding.astar(maze, pos, EXIT_POS, avoid=False))
            return self.state, path

        # ── 2. Time pressure ──────────────────────────────────────────────────
        if time_rem < CRITICAL_TIME:
            if self.state != AIDecision.ESCAPING or not cur_path:
                self.state = AIDecision.ESCAPING
                path = (Pathfinding.astar(maze, pos, EXIT_POS, ghosts) or
                        Pathfinding.astar(maze, pos, EXIT_POS, avoid=False))
                return self.state, path
            return self.state, None

        # ── Periodic recompute ────────────────────────────────────────────────
        self.countdown -= 1
        if self.countdown > 0 and cur_path:
            return self.state, None

        self.countdown = 30

        # ── 3. Collect best pallet ────────────────────────────────────────────
        best = self._best_pallet(pos, pallets, ghosts)
        if best:
            self.target = best.grid_pos
            path = Pathfinding.astar(maze, pos, self.target, ghosts)
            if path:
                self.state = AIDecision.COLLECTING_PALLET
            else:
                path = Pathfinding.astar(maze, pos, self.target, avoid=False)
                self.state = AIDecision.REROUTING
            return self.state, path

        # ── 4. Escape ─────────────────────────────────────────────────────────
        self.target = EXIT_POS
        self.state  = AIDecision.ESCAPING
        path = (Pathfinding.astar(maze, pos, EXIT_POS, ghosts) or
                Pathfinding.astar(maze, pos, EXIT_POS, avoid=False))
        return self.state, path

    def _nearest_ghost(self, pos: Tuple, ghosts: list) -> float:
        if not ghosts: return float('inf')
        return min(abs(pos[0]-g.grid_pos[0])+abs(pos[1]-g.grid_pos[1]) for g in ghosts)

    def _best_pallet(self, pos, pallets, ghosts) -> Optional['Pallet']:
        best, bs = None, float('-inf')
        for p in pallets:
            if p.collected: continue
            d = abs(pos[0]-p.col)+abs(pos[1]-p.row)
            gd = self._nearest_ghost((p.col,p.row), ghosts)
            pen = max(0,(DANGER_RADIUS-gd)*15) if gd < DANGER_RADIUS else 0
            score = -d-pen
            if score > bs: bs=score; best=p
        return best

# ══════════════════════════════════════════════════════════════════════════════
#  SENSOR SYSTEM
# ══════════════════════════════════════════════════════════════════════════════

class SensorSystem:
    """
    Simulated VL53L1X Time-of-Flight sensor array (Front/Left/Right/Back).
    Raycasts from PacBot grid position until hitting a wall.
    Readings are in centimetres (TILE_CM per tile).

    Hardware mapping:
      ESP32 GPIO → I²C bus → VL53L1X × 4
      XSHUT pins select individual sensors on the shared I²C bus.
    """

    MAX = 10   # max range in tiles

    def __init__(self):
        self.front = self.left = self.right = self.back = self.ghost = 0.0

    def _cast(self, c: int, r: int, dc: int, dr: int, maze: 'Maze') -> float:
        for t in range(1, self.MAX+1):
            c += dc; r += dr
            if maze.is_wall(c, r):
                return t * TILE_CM
        return self.MAX * TILE_CM

    def update(self, pacbot: 'PacBot', maze: 'Maze', ghosts: list):
        c, r = pacbot.grid_pos
        dx, dy = pacbot.dir_x, pacbot.dir_y
        if dx==0 and dy==0: dx=1
        self.front = self._cast(c, r,  dx,  dy, maze)
        self.back  = self._cast(c, r, -dx, -dy, maze)
        self.left  = self._cast(c, r,  dy, -dx, maze)
        self.right = self._cast(c, r, -dy,  dx, maze)
        if ghosts:
            mp = min(math.hypot(g.px-pacbot.px, g.py-pacbot.py) for g in ghosts)
            self.ghost = mp/TILE*TILE_CM
        else:
            self.ghost = 999.0

# ══════════════════════════════════════════════════════════════════════════════
#  GAME MANAGER
# ══════════════════════════════════════════════════════════════════════════════

class GameManager:
    """
    Master controller: init, game loop, rendering, scoring, state transitions.

    State machine:
        TITLE → (key/click) → PLAYING → WIN | LOSE → (R key) → PLAYING
    """

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("PacBot – Autonomous Maze Simulation")
        self.screen = pygame.display.set_mode((WIN_W, WIN_H))
        self.clock  = pygame.time.Clock()

        self.f_xl = pygame.font.SysFont("Consolas", 44, bold=True)
        self.f_lg = pygame.font.SysFont("Consolas", 24, bold=True)
        self.f_md = pygame.font.SysFont("Consolas", 16, bold=True)
        self.f_sm = pygame.font.SysFont("Consolas", 13)

        self.state = GameState.TITLE
        self.mode  = GameMode.AUTONOMOUS
        self.ox    = 0           # maze draw X offset
        self.oy    = TOP_H       # maze draw Y offset

        # Button rects (set during draw; used for click detection)
        self._btn_manual  = pygame.Rect(0,0,1,1)
        self._btn_auto    = pygame.Rect(0,0,1,1)
        self._btn_toggle  = pygame.Rect(0,0,1,1)
        self._btn_restart = pygame.Rect(0,0,1,1)

        self._init()

    # ── Entity reset ──────────────────────────────────────────────────────────
    def _init(self):
        self.maze    = Maze()
        self.pacbot  = PacBot(*PACBOT_START)
        self.ghosts  = [
            Ghost(name, GHOST_COLORS[name], wps, self.maze)
            for name, wps in GHOST_ROUTES.items()
        ]
        self.pallets = [
            Pallet(c, r)
            for r in range(ROWS) for c in range(COLS)
            if not self.maze.is_wall(c,r)
            and (c,r) not in (PACBOT_START, EXIT_POS)
        ]
        self.total_pallets   = len(self.pallets)
        self.decision_engine = DecisionEngine()
        self.sensors         = SensorSystem()
        self.ai_path:    List[Tuple[int,int]] = []
        self.ai_decision: AIDecision          = AIDecision.IDLE
        self.time_rem   : float               = float(GAME_TIME)
        self.score      : int                 = 0

    # ── Main loop ─────────────────────────────────────────────────────────────
    def run(self):
        while True:
            dt = self.clock.tick(FPS)/1000.0
            self._events()
            if self.state == GameState.PLAYING:
                self._update(dt)
            self._draw()
            pygame.display.flip()

    # ── Events ────────────────────────────────────────────────────────────────
    def _events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif ev.type == pygame.KEYDOWN:
                self._on_key(ev.key)
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                self._on_click(ev.pos)

    def _on_key(self, key: int):
        if key == pygame.K_ESCAPE:
            pygame.quit(); sys.exit()

        if self.state == GameState.TITLE:
            if key in (pygame.K_RETURN, pygame.K_SPACE): self._start()
            elif key == pygame.K_1: self.mode=GameMode.MANUAL; self._start()
            elif key == pygame.K_2: self.mode=GameMode.AUTONOMOUS; self._start()

        elif self.state == GameState.PLAYING:
            if key == pygame.K_TAB:
                self.mode = (GameMode.MANUAL
                             if self.mode==GameMode.AUTONOMOUS
                             else GameMode.AUTONOMOUS)
                self.ai_path=[]; self.decision_engine.countdown=0
            elif key == pygame.K_r:
                self._init(); self.state=GameState.PLAYING

        elif self.state in (GameState.WIN, GameState.LOSE):
            if key in (pygame.K_r, pygame.K_RETURN):
                self._init(); self.state=GameState.PLAYING

    def _on_click(self, pos):
        if self.state == GameState.TITLE:
            if self._btn_manual.collidepoint(pos):
                self.mode=GameMode.MANUAL; self._start()
            elif self._btn_auto.collidepoint(pos):
                self.mode=GameMode.AUTONOMOUS; self._start()
        elif self.state in (GameState.WIN, GameState.LOSE):
            if self._btn_restart.collidepoint(pos):
                self._init(); self.state=GameState.PLAYING
        elif self.state == GameState.PLAYING:
            if self._btn_toggle.collidepoint(pos):
                self.mode = (GameMode.MANUAL
                             if self.mode==GameMode.AUTONOMOUS
                             else GameMode.AUTONOMOUS)
                self.ai_path=[]; self.decision_engine.countdown=0

    def _start(self):
        self._init(); self.state=GameState.PLAYING

    # ── Game update ───────────────────────────────────────────────────────────
    def _update(self, dt: float):
        # Timer
        self.time_rem -= dt
        if self.time_rem <= 0:
            self.time_rem=0; self.state=GameState.LOSE; return

        # Ghosts
        for g in self.ghosts:
            g.update(dt)

        # PacBot movement
        if self.mode == GameMode.MANUAL:
            keys = pygame.key.get_pressed()
            self.pacbot.handle_input(keys, self.maze)
            self.pacbot.update_manual(dt, self.maze)
        else:
            # Autonomous: ask decision engine
            decision, new_path = self.decision_engine.update(
                self.pacbot, self.maze, self.ghosts,
                self.pallets, self.time_rem, self.ai_path)
            self.ai_decision = decision
            if new_path is not None:
                self.ai_path = new_path
            self.pacbot.path = self.ai_path          # for path overlay draw
            self.pacbot.update_autonomous(dt, self.maze, self.ai_path)

        # Sensors
        self.sensors.update(self.pacbot, self.maze, self.ghosts)

        # Pallet collection
        pg = self.pacbot.grid_pos
        for p in self.pallets:
            if not p.collected and p.grid_pos == pg:
                p.collected=True; self.score+=10
                self.pacbot.pallets+=1; self.pacbot.score+=10

        # Win: reached exit
        if pg == EXIT_POS:
            self.score += int(self.time_rem)*5
            self.state=GameState.WIN; return

        # Lose: ghost collision (pixel distance)
        for g in self.ghosts:
            if math.hypot(g.px-self.pacbot.px, g.py-self.pacbot.py) < TILE*0.75:
                self.state=GameState.LOSE; return

    # ══════════════════════════════════════════════════════════════════════════
    #  RENDERING
    # ══════════════════════════════════════════════════════════════════════════
    def _draw(self):
        self.screen.fill(C_BG)
        if self.state == GameState.TITLE:
            self._draw_title()
        else:
            self._draw_game()
            if self.state == GameState.WIN:
                self._draw_end(won=True)
            elif self.state == GameState.LOSE:
                self._draw_end(won=False)

    # ── Game scene ────────────────────────────────────────────────────────────
    def _draw_game(self):
        ox, oy = self.ox, self.oy
        self.maze.draw(self.screen, ox, oy, self.f_sm)
        for g in self.ghosts:
            g.draw_danger_zone(self.screen, ox, oy)
        for p in self.pallets:
            p.draw(self.screen, ox, oy)
        for g in self.ghosts:
            g.draw(self.screen, ox, oy)
        self.pacbot.draw(self.screen, ox, oy)
        self._draw_top()
        self._draw_panel()
        self._draw_bottom()

    # ── Top HUD bar ───────────────────────────────────────────────────────────
    def _draw_top(self):
        pygame.draw.rect(self.screen, C_TOP_BG, (0,0,WIN_W,TOP_H))
        pygame.draw.line(self.screen, C_ACCENT, (0,TOP_H-1),(WIN_W,TOP_H-1),2)

        # Brand
        self.screen.blit(self.f_lg.render("PacBot", True, C_YELLOW), (10,18))

        # Score
        self._hud_item(160, "SCORE", str(self.score), C_YELLOW)

        # Time
        t = int(self.time_rem)
        tc = C_RED if t<CRITICAL_TIME else C_GREEN
        self._hud_item(310, "TIME", f"{t}s", tc)

        # Pallets
        pc = sum(1 for p in self.pallets if p.collected)
        self._hud_item(440, "PALLETS", f"{pc}/{self.total_pallets}", C_PALLET)

        # Lives / mode indicator
        mode_txt = "AUTO" if self.mode==GameMode.AUTONOMOUS else "MANUAL"
        mc = C_ACCENT if self.mode==GameMode.AUTONOMOUS else C_ORANGE
        btn = pygame.Rect(WIN_W-210, 12, 90, 34)
        pygame.draw.rect(self.screen, mc, btn, border_radius=6)
        pygame.draw.rect(self.screen, C_WHITE, btn, 1, border_radius=6)
        bt = self.f_md.render(mode_txt, True, C_BLACK)
        self.screen.blit(bt, bt.get_rect(center=btn.center))
        self._btn_toggle = btn
        self.screen.blit(self.f_sm.render("[TAB]",True,C_DGRAY),(WIN_W-110,22))

    def _hud_item(self, x:int, label:str, value:str, col):
        self.screen.blit(self.f_sm.render(label, True, C_GRAY),  (x, 8))
        self.screen.blit(self.f_md.render(value, True, col),      (x, 26))

    # ── Right side panel ──────────────────────────────────────────────────────
    def _draw_panel(self):
        px = MAZE_W;  py = TOP_H;  pw = PANEL_W;  ph = MAZE_H
        pygame.draw.rect(self.screen, C_PANEL_BG, (px, py, pw, ph))
        pygame.draw.line(self.screen, C_PANEL_SEP, (px,py),(px,py+ph), 2)

        cx = px+10
        y  = py+8

        def section(title, col=C_ACCENT):
            nonlocal y
            pygame.draw.line(self.screen,C_PANEL_SEP,(cx,y),(cx+pw-16,y))
            y += 5
            self.screen.blit(self.f_md.render(title,True,col),(cx,y))
            y += 22

        def row(label, val, vc=C_WHITE):
            nonlocal y
            self.screen.blit(self.f_sm.render(label+":", True, C_GRAY),(cx,y))
            self.screen.blit(self.f_sm.render(str(val),  True, vc),    (cx+pw-130,y))
            y += 18

        # ── AI Decision ───────────────────────────────────────────────────────
        section("AI DECISION", C_ACCENT)
        if self.mode == GameMode.MANUAL:
            dec_txt = "Manual Control"
            dc = C_ORANGE
        else:
            dec_txt = self.ai_decision.value
            dc = {AIDecision.COLLECTING_PALLET:C_GREEN,
                  AIDecision.AVOIDING_GHOST:   C_RED,
                  AIDecision.REROUTING:        C_ORANGE,
                  AIDecision.ESCAPING:         C_YELLOW,
                  AIDecision.IDLE:             C_GRAY,
                  AIDecision.STUCK:            C_ORANGE
                  }.get(self.ai_decision, C_WHITE)

        # Draw decision in a highlighted box
        dec_surf = self.f_md.render(dec_txt, True, dc)
        dec_bg   = pygame.Rect(cx, y, pw-16, 24)
        pygame.draw.rect(self.screen, C_DGRAY, dec_bg, border_radius=4)
        self.screen.blit(dec_surf, (cx+4, y+3))
        y += 30

        tgt = self.decision_engine.target if self.mode==GameMode.AUTONOMOUS else "---"
        row("Target", tgt, C_PATH)
        row("Path",   f"{len(self.ai_path)} steps" if self.mode==GameMode.AUTONOMOUS else "---", C_GRAY)
        y += 4

        # ── Ghost Status ──────────────────────────────────────────────────────
        section("GHOST STATUS", GHOST_COLORS['Blinky'])
        for g in self.ghosts:
            gc,gr = g.grid_pos
            d = abs(self.pacbot.grid_pos[0]-gc)+abs(self.pacbot.grid_pos[1]-gr)
            threat = "CRITICAL" if d<=3 else ("NEAR" if d<=DANGER_RADIUS else "safe")
            tc = C_RED if d<=3 else (C_ORANGE if d<=DANGER_RADIUS else C_GREEN)
            # Draw coloured ghost indicator dot
            pygame.draw.circle(self.screen, g.color, (cx+4, y+8), 5)
            self.screen.blit(self.f_sm.render(g.name, True, g.color), (cx+14, y))
            self.screen.blit(self.f_sm.render(threat, True, tc), (cx+pw-130, y))
            y += 18
        y += 4

        # ── Sensor Readings ───────────────────────────────────────────────────
        section("SENSOR READINGS", C_GREEN)
        s = self.sensors
        sdata = [("Front",s.front),("Left",s.left),
                 ("Right",s.right),("Back",s.back)]
        for lbl,val in sdata:
            vc = C_ORANGE if val < TILE_CM*2 else C_GREEN
            row(lbl, f"{val:.0f} cm", vc)
        gc2 = C_RED if s.ghost < DANGER_RADIUS*TILE_CM else C_GREEN
        row("Ghost", f"{s.ghost:.0f} cm", gc2)
        y += 4

        # ── Hardware Architecture ─────────────────────────────────────────────
        section("HARDWARE MAP", C_YELLOW)
        hw = [("MCU","ESP32 WROOM-32"),("Sensor","VL53L1X ToF x4"),
              ("Driver","TB6612FNG"),("Motor","N20+Encoder"),("Wheel","N20 Rubber")]
        for comp,desc in hw:
            self.screen.blit(self.f_sm.render(comp+":", True, C_ACCENT), (cx,y))
            self.screen.blit(self.f_sm.render(desc,     True, C_GRAY),   (cx+60,y))
            y += 17
        y += 4

        # ── Statistics ────────────────────────────────────────────────────────
        section("STATISTICS", C_ACCENT)
        pc = sum(1 for p in self.pallets if p.collected)
        row("Collected", f"{pc} pallets", C_PALLET)
        row("Score",     str(self.score), C_YELLOW)
        tc2 = C_RED if self.time_rem < CRITICAL_TIME else C_GREEN
        row("Time Left", f"{self.time_rem:.1f}s", tc2)

    # ── Bottom bar ────────────────────────────────────────────────────────────
    def _draw_bottom(self):
        by = TOP_H+MAZE_H
        pygame.draw.rect(self.screen, C_TOP_BG, (0,by,WIN_W,BOT_H))
        pygame.draw.line(self.screen, C_ACCENT, (0,by),(WIN_W,by),1)
        mode_str = "AUTONOMOUS" if self.mode==GameMode.AUTONOMOUS else "MANUAL"
        hints = f"  {mode_str} MODE  |  [TAB] Toggle  |  [R] Restart  |  [ESC] Quit"
        self.screen.blit(self.f_sm.render(hints,True,C_GRAY),(10,by+15))
        pos_txt = f"PacBot: {self.pacbot.grid_pos}"
        ps = self.f_sm.render(pos_txt,True,C_DGRAY)
        self.screen.blit(ps,(WIN_W-ps.get_width()-10,by+15))

    # ── Title screen ──────────────────────────────────────────────────────────
    def _draw_title(self):
        # Animated grid bg
        t = pygame.time.get_ticks()/1000.0
        for c in range(0,WIN_W,TILE):
            for r in range(0,WIN_H,TILE):
                if (c//TILE+r//TILE)%3==0:
                    pygame.draw.rect(self.screen,(10,10,40),(c,r,TILE-1,TILE-1))

        cy = WIN_H//2-200
        # Title
        t1 = self.f_xl.render("PAC", True, C_YELLOW)
        t2 = self.f_xl.render("BOT", True, C_ACCENT)
        total_w = t1.get_width()+t2.get_width()+8
        tx = WIN_W//2-total_w//2
        self.screen.blit(t1,(tx,cy))
        self.screen.blit(t2,(tx+t1.get_width()+8,cy))
        cy += 60
        sub = self.f_lg.render("Autonomous Maze Simulation", True, C_GRAY)
        self.screen.blit(sub,(WIN_W//2-sub.get_width()//2,cy))
        cy += 50

        descs = [
            "Navigate · Collect Pallets · Avoid Ghosts · Reach the EXIT",
            "AI powered by A* pathfinding with ghost-danger weighting",
            "Simulates an ESP32-based real-world robot"
        ]
        for d in descs:
            s = self.f_sm.render(d,True,C_DGRAY)
            self.screen.blit(s,(WIN_W//2-s.get_width()//2,cy)); cy+=20
        cy += 24

        # Mode buttons
        for i,(txt,mode,col) in enumerate([
            ("  1   MANUAL MODE  ","M",C_ORANGE),
            ("  2  AUTONOMOUS  AI","A",C_ACCENT)]):
            bw,bh = 260,52
            bx = WIN_W//2-bw-12+i*(bw+24)
            btn = pygame.Rect(bx,cy,bw,bh)
            pygame.draw.rect(self.screen, col, btn, border_radius=10)
            pygame.draw.rect(self.screen, C_WHITE, btn, 2, border_radius=10)
            bt = self.f_md.render(txt,True,C_BLACK)
            self.screen.blit(bt,bt.get_rect(center=btn.center))
            if mode=="M": self._btn_manual=btn
            else:         self._btn_auto=btn
        cy += 72

        # Ghost legend
        legend_x = WIN_W//2 - 200
        pygame.draw.rect(self.screen,C_DGRAY,(legend_x,cy,400,26),border_radius=6)
        lx = legend_x+8
        for name,col in GHOST_COLORS.items():
            pygame.draw.circle(self.screen,col,(lx+6,cy+13),6)
            ns=self.f_sm.render(name,True,col)
            self.screen.blit(ns,(lx+16,cy+5))
            lx += 100
        cy += 40

        # Blink prompt
        if (pygame.time.get_ticks()//600)%2==0:
            ps=self.f_sm.render("Press ENTER or click a button to start",True,C_DGRAY)
            self.screen.blit(ps,(WIN_W//2-ps.get_width()//2,cy))

    # ── End screen overlay ────────────────────────────────────────────────────
    def _draw_end(self, won: bool):
        overlay = pygame.Surface((WIN_W,WIN_H),pygame.SRCALPHA)
        overlay.fill((0,0,0,170))
        self.screen.blit(overlay,(0,0))

        cx = WIN_W//2;  cy = WIN_H//2-110
        if won:
            title = self.f_xl.render("MISSION COMPLETE!", True, C_GREEN)
        else:
            reason = "caught by a Ghost!" if self.time_rem>0 else "Time expired!"
            title = self.f_xl.render("MISSION FAILED", True, C_RED)
        self.screen.blit(title, title.get_rect(centerx=cx).move(0,cy))
        cy += 66

        pc = sum(1 for p in self.pallets if p.collected)
        time_bonus = int(self.time_rem)*5 if won else 0
        lines = [
            (f"Final Score  :  {self.score}", C_YELLOW),
            (f"Pallets      :  {pc} / {self.total_pallets}", C_PALLET),
            (f"Time Bonus   :  +{time_bonus}", C_GREEN),
            (f"Time Left    :  {self.time_rem:.1f}s",
             C_GREEN if self.time_rem>10 else C_RED),
        ]
        if not won and self.time_rem>0:
            lines.insert(1, ("Ghost caught PacBot!", C_RED))
        for txt,col in lines:
            s=self.f_md.render(txt,True,col)
            self.screen.blit(s,s.get_rect(centerx=cx).move(0,cy)); cy+=30

        cy += 14
        btn=pygame.Rect(cx-110,cy,220,48)
        bc = C_GREEN if won else C_RED
        pygame.draw.rect(self.screen,bc,btn,border_radius=10)
        pygame.draw.rect(self.screen,C_WHITE,btn,2,border_radius=10)
        bt=self.f_md.render("[R]  RESTART",True,C_BLACK)
        self.screen.blit(bt,bt.get_rect(center=btn.center))
        self._btn_restart=btn

# ══════════════════════════════════════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    game = GameManager()
    game.run()

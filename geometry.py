#!/usr/bin/env python3
# geometry.py - Maze geometry constants

CELL_CM      = 33.2          # one grid square
COLS, ROWS   = 10, 5         # width × height
WALL_THICKCM = 2.0           # measured once with a caliper
START_CELL   = (0, ROWS-1)   # (0,4) – red arrow
EXIT_CELL    = (COLS-1, 0)   # (9,0) – green arrow

# Direction vectors for moving in each absolute direction
# North(0), East(1), South(2), West(3)
DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0] 
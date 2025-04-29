#!/usr/bin/env python3
# pose.py - Robot pose tracking

from dataclasses import dataclass
from typing import Tuple, Literal

@dataclass(slots=True)
class Pose:
    """Tracks the robot's position and orientation in the maze"""
    cell: Tuple[int, int]  # (x, y) coordinate in grid cells
    heading: Literal[0, 1, 2, 3]  # 0=North, 1=East, 2=South, 3=West
    mm_into_cell: float = 0.0  # How far into the current cell (0-332mm)
    
    def move_forward(self, distance_mm: float):
        """Update position when moving forward"""
        self.mm_into_cell += distance_mm
    
    def reset_cell_position(self):
        """Reset position within cell after reaching cell center"""
        self.mm_into_cell = 0.0
    
    def update_after_straight(self, cells: int = 1):
        """Update cell position after moving straight by a number of cells"""
        from geometry import DX, DY
        
        x, y = self.cell
        dx, dy = DX[self.heading], DY[self.heading]
        
        self.cell = (x + dx * cells, y + dy * cells)
        self.reset_cell_position()
    
    def update_after_turn(self, left: bool = True):
        """Update heading after turning left or right"""
        if left:
            self.heading = (self.heading - 1) % 4
        else:
            self.heading = (self.heading + 1) % 4 
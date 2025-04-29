import unittest
import time
from maze_solver import MazeSolver, Dir

class TestMazeSolverMovement(unittest.TestCase):

    def setUp(self):
        """Set up the MazeSolver in simulation mode for each test."""
        self.solver = MazeSolver()
        self.solver.set_simulation_mode(True)
        # Reset position and direction before each test
        self.solver.current_position = (0, 0)
        self.solver.current_direction = 0  # Start facing North

    def test_go_straight_north(self):
        """Test moving straight one cell when facing North."""
        initial_pos = (0, 0)
        initial_dir = 0 # North
        self.solver.current_position = initial_pos
        self.solver.current_direction = initial_dir
        self.solver.go_straight()
        self.assertEqual(self.solver.current_position, (0, 1))
        self.assertEqual(self.solver.current_direction, initial_dir) # Direction shouldn't change

    def test_go_straight_east(self):
        """Test moving straight one cell when facing East."""
        initial_pos = (1, 1)
        initial_dir = 1 # East
        self.solver.current_position = initial_pos
        self.solver.current_direction = initial_dir
        self.solver.go_straight()
        self.assertEqual(self.solver.current_position, (2, 1))
        self.assertEqual(self.solver.current_direction, initial_dir)

    def test_go_straight_multiple_cells(self):
        """Test moving straight multiple cells."""
        initial_pos = (0, 0)
        initial_dir = 2 # South
        self.solver.current_position = initial_pos
        self.solver.current_direction = initial_dir
        self.solver.go_straight(cells=3)
        # Moving 3 cells South from (0,0) -> (0,-1) -> (0,-2) -> (0,-3)
        self.assertEqual(self.solver.current_position, (0, -3))
        self.assertEqual(self.solver.current_direction, initial_dir)

    def test_turn_left(self):
        """Test turning 90 degrees left."""
        # North (0) -> West (3)
        self.solver.current_direction = 0
        self.solver.turn(left=True)
        self.assertEqual(self.solver.current_direction, 3)
        # West (3) -> South (2)
        self.solver.turn(left=True)
        self.assertEqual(self.solver.current_direction, 2)
        # South (2) -> East (1)
        self.solver.turn(left=True)
        self.assertEqual(self.solver.current_direction, 1)
        # East (1) -> North (0)
        self.solver.turn(left=True)
        self.assertEqual(self.solver.current_direction, 0)

    def test_turn_right(self):
        """Test turning 90 degrees right."""
        # North (0) -> East (1)
        self.solver.current_direction = 0
        self.solver.turn(left=False)
        self.assertEqual(self.solver.current_direction, 1)
        # East (1) -> South (2)
        self.solver.turn(left=False)
        self.assertEqual(self.solver.current_direction, 2)
        # South (2) -> West (3)
        self.solver.turn(left=False)
        self.assertEqual(self.solver.current_direction, 3)
        # West (3) -> North (0)
        self.solver.turn(left=False)
        self.assertEqual(self.solver.current_direction, 0)

    def test_turn_180(self):
        """Test turning 180 degrees (as done in move(Dir.BACK))."""
        initial_dir = 1 # East
        self.solver.current_direction = initial_dir
        # Simulate the two turns from move(Dir.BACK) which uses left turns
        self.solver.turn(left=True)
        self.solver.turn(left=True)
        expected_dir = (initial_dir - 2) % 4 # East (1) -> North (0) -> West (3)
        self.assertEqual(self.solver.current_direction, expected_dir)


if __name__ == '__main__':
    unittest.main() 
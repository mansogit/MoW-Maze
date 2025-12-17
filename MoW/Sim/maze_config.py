"""
Maze Configuration - Edit this file to change the maze
"""

ROWS = 3
COLS = 4

START = (0, 0)  # (x, y)
END = (3, 2)    # (x, y)

# Walls between adjacent blocks: ((x1, y1), (x2, y2))
WALLS = [
    ((0, 0), (1, 0)),
    ((0, 1), (1, 1)),
    ((2, 0), (2, 1)),
    ((1, 2), (2, 2)),
    ((2, 1), (3, 1)),
    ((2, 2), (3, 2)),
]
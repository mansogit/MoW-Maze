# MoW

## Directory Structure

| Folder | Description |
|--------|-------------|
| `Tested/` | Scripts verified and validated on physical hardware |
| `Sim/` | Simulation scripts and code tested in simulated environments |

## Tested Scripts

### Tester7Coord.py
Coordinate-based maze navigation for the drone. Uses absolute coordinates to move between blocks in a grid, with obstacle detection and path optimization based on visit counts and Manhattan distance to goal.

### aMaze3.py
Enhanced maze navigation with real-time visualization. Builds on coordinate-based movement and adds a matplotlib display showing the drone's position, path history, and detected walls as the maze is explored.

# 2D Car Simulator

A 2D car simulation built with Python and Pygame featuring realistic car physics, A* pathfinding, and automated parking.

## Features

- **Realistic Car Physics**: Differential drive simulation with wheel-based movement
- **A* Pathfinding**: Intelligent route planning to parking spaces
- **Automated Parking**: Autonomous navigation to available parking spots
- **Interactive Controls**: Manual car control with keyboard input
- **Collision Detection**: Obstacle avoidance and boundary checking
- **Performance Optimized**: Efficient rendering and physics calculations

## Requirements

- Python 3.8+
- pygame
- numpy
- scipy

## Installation

1. Clone the repository:
```bash
git clone https://github.com/HLRas/2D-Simulator.git
cd 2D-Simulator
```

2. Create a virtual environment:
```bash
python -m venv .venv
```

3. Activate the virtual environment:
   - Windows: `.venv\Scripts\activate`
   - macOS/Linux: `source .venv/bin/activate`

4. Install dependencies:
```bash
pip install pygame numpy scipy
```

## Usage

Run the simulation:
```bash
python main.py
```

### Controls

- **Arrow Keys**: Manual car control (acceleration, turning)
- **Q**: Set start position at car location
- **A**: Auto-navigate to nearest parking space
- **C**: Clear current path and stop autopilot
- **SPACE**: Stop autopilot manually
- **F1**: Display debug information

## Project Structure

- `main.py`: Main game loop and event handling
- `car.py`: Car physics and movement logic
- `map_generation.py`: Map rendering and obstacle management
- `astar_pathfind.py`: A* pathfinding algorithm implementation
- `parking_space.py`: Parking space management
- `config.py`: Configuration constants and settings
- `cubes.py`: Grid-based world representation

## Configuration

Modify `config.py` to adjust:
- Screen dimensions
- Car physics parameters
- Pathfinding settings
- Visual appearance

## License

This project is open source and available under the [MIT License](LICENSE).

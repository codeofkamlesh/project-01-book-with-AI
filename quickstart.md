# Quickstart Guide: AI/Spec-Driven Book Architecture

## Overview

This guide provides a quick introduction to setting up and working with the 4-model humanoid robotics book architecture (ROS2 → Simulation → Isaac → VLA). Follow these steps to get started with the book's content, examples, and validation systems.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (primary), Windows 10/11 or macOS (secondary)
- **CPU**: 8+ cores recommended
- **RAM**: 32GB+ recommended
- **GPU**: NVIDIA RTX 3080+ (for Isaac Sim and VLA models) or equivalent
- **Storage**: 50GB+ free space for all simulation environments

### Software Dependencies
1. **ROS 2**: Humble Hawksbill (or Foxy for compatibility)
2. **Gazebo**: Harmonic or Garden (matching ROS 2 version)
3. **Unity**: 2022.3 LTS (with Robotics package)
4. **NVIDIA Isaac Sim**: 2023.1+ (requires NVIDIA GPU)
5. **Python**: 3.8+ with pip
6. **Node.js**: 16+ (for Docusaurus documentation)
7. **Git**: Version control
8. **Docker**: For containerized examples (optional but recommended)

## Setup Process

### 1. Environment Setup
```bash
# Clone the repository
git clone <repository-url>
cd project-01-book-with-AI

# Install system dependencies
sudo apt update
sudo apt install python3-pip python3-venv build-essential
```

### 2. Python Environment Setup
```bash
# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. ROS 2 Setup
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Adjust for your ROS 2 installation

# Create workspace
mkdir -p ~/book_ws/src
cd ~/book_ws

# Build workspace
colcon build --symlink-install
source install/setup.bash
```

### 4. Simulation Environment Setup

#### Gazebo Setup
```bash
# Install Gazebo (already included with ROS 2 Humble)
# Verify installation
gz sim --version
```

#### Unity Setup
1. Download Unity Hub from unity.com
2. Install Unity 2022.3 LTS
3. Install Unity Robotics package via Package Manager
4. Import robotics examples

#### Isaac Sim Setup
1. Install NVIDIA Isaac Sim from developer.nvidia.com
2. Ensure NVIDIA GPU drivers are up to date
3. Verify CUDA compatibility
4. Install Isaac ROS packages

## Book Structure

### 4-Model Learning Path
1. **ROS2 Foundations** - Core ROS 2 concepts, nodes, topics, services
2. **Simulation** - Gazebo and Unity environments, physics, visualization
3. **Isaac** - NVIDIA Isaac Sim, Isaac ROS GEMs, perception pipelines
4. **VLA** - Vision-Language-Action systems, multimodal AI for humanoid control

## Running Examples

### Basic ROS 2 Example
```bash
# Navigate to ROS 2 examples
cd examples/ros2/

# Run a publisher/subscriber example
python3 publisher.py &
python3 subscriber.py
```

### Gazebo Simulation
```bash
# Launch a basic robot simulation
cd examples/simulation/gazebo/
gz sim -r -v 4 basic_robot.sdf
```

### Isaac Sim Example
```bash
# Launch Isaac Sim with a humanoid robot
cd examples/isaac/
# Run the Isaac Sim example (requires Isaac Sim installation)
python3 humanoid_control.py
```

### VLA Pipeline
```bash
# Run a basic VLA example
cd examples/vla/
python3 prompt_to_action.py --prompt "pick up the red cube"
```

## Validation and Testing

### Run All Code Examples
```bash
# Validate all examples in the book
python3 scripts/validate_examples.py

# Validate specific model examples
python3 scripts/validate_examples.py --model ros2
python3 scripts/validate_examples.py --model simulation
python3 scripts/validate_examples.py --model isaac
python3 scripts/validate_examples.py --model vla
```

### Build Documentation
```bash
# Navigate to Docusaurus directory
cd docusaurus/

# Install dependencies
npm install

# Build the site
npm run build

# Serve locally
npm run serve
```

## Key Directories

```
project-root/
├── docs/                    # Docusaurus documentation
├── examples/               # Runnable code examples
│   ├── ros2/              # ROS 2 examples
│   ├── simulation/        # Gazebo/Unity examples
│   ├── isaac/             # Isaac Sim examples
│   └── vla/               # VLA examples
├── diagrams/               # Draw.io compatible diagrams
├── research/               # Collected documentation
├── specs/                  # Feature specifications
├── tests/                  # Validation scripts
└── scripts/                # Utility scripts
```

## Common Commands

### Development
```bash
# Create a new chapter
python3 scripts/create_chapter.py --title "New Chapter" --model ros2

# Validate a specific chapter
python3 scripts/validate_chapter.py --chapter 001-ros2-foundations

# Run all validations
python3 scripts/run_validations.py
```

### Documentation
```bash
# Build documentation locally
cd docusaurus && npm run build

# Serve documentation with live reload
cd docusaurus && npm run start

# Deploy to GitHub Pages
cd docusaurus && npm run deploy
```

## Troubleshooting

### Common Issues

1. **ROS 2 Environment Not Found**
   - Solution: Ensure ROS 2 is properly sourced: `source /opt/ros/humble/setup.bash`

2. **Gazebo Not Launching**
   - Solution: Check graphics drivers and X11 forwarding if using SSH

3. **Isaac Sim GPU Error**
   - Solution: Verify NVIDIA GPU drivers and CUDA installation

4. **Python Package Issues**
   - Solution: Use virtual environment and install from requirements.txt

### Getting Help
- Check the `docs/troubleshooting.md` file
- Review the issue tracker in the repository
- Consult the relevant official documentation for each technology

## Next Steps

1. Start with the **ROS2 Foundations** chapter to understand core concepts
2. Progress through the 4-model sequence for comprehensive learning
3. Try the hands-on examples in each chapter
4. Validate your understanding by running the provided examples
5. Contribute to the book by reporting issues or suggesting improvements

## Contributing

To contribute to the book:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Run validation scripts
5. Submit a pull request with your improvements

For detailed contribution guidelines, see `CONTRIBUTING.md`.
# AI/Spec-Driven Robotics Book

This repository contains the complete implementation of the AI/Spec-Driven Robotics Book, following a 4-model architecture: ROS2 → Simulation → Isaac → VLA.

## Book Structure

The book is organized into four main models:

1. **ROS2 Foundations** - Core concepts of Robot Operating System 2
2. **Simulation** - Gazebo and Unity simulation environments
3. **NVIDIA Isaac** - Isaac Sim and Isaac ROS for perception and control
4. **Vision-Language-Action (VLA)** - Multimodal AI for humanoid control

## Repository Structure

```
project-root/
├── docs/                    # Docusaurus documentation source
│   ├── ros2-foundations/    # ROS 2 concepts and architecture
│   ├── simulation/          # Gazebo & Unity simulation
│   ├── nvidia-isaac/        # Isaac Sim and Isaac ROS
│   └── vla-humanoids/       # Vision-Language-Action systems
├── specs/                   # Feature specifications
├── research/                # Collected documentation and papers
├── diagrams/                # Draw.io compatible diagrams
├── examples/                # Runnable code examples
│   ├── ros2/               # ROS 2 examples
│   ├── simulation/         # Simulation examples
│   ├── isaac/              # Isaac Sim examples
│   └── vla/                # VLA examples
├── src/                     # Supporting scripts and utilities
├── tests/                   # Validation and testing scripts
├── docusaurus/              # Docusaurus site configuration
└── history/                 # Prompt History Records
```

## Getting Started

### Prerequisites

- Ubuntu 22.04 LTS (recommended)
- Python 3.8+
- ROS 2 Humble Hawksbill
- Git

### Setup

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd project-01-book-with-AI
   ```

2. Install Python dependencies:
   ```bash
   python3 -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

3. Set up ROS 2 workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   mkdir -p ~/book_ws/src
   cd ~/book_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

### Running Examples

Each model includes runnable examples in the `examples/` directory:

```bash
# Navigate to ROS2 examples
cd examples/ros2/

# Run a publisher/subscriber example
python3 publisher_subscriber/minimal_publisher.py &
python3 publisher_subscriber/minimal_subscriber.py
```

## Docusaurus Site

The book is built using Docusaurus. To run the documentation site locally:

```bash
cd docusaurus/
npm install
npm run start
```

## Contributing

This project follows the Spec-Kit Plus spec-driven workflow. All changes should be made through the proper specification and planning process.

## License

This project is licensed under the Apache 2.0 License - see the LICENSE file for details.
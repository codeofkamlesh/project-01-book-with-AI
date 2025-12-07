"""
SimulationEnvironment model representing a simulation environment used in the book's examples.
"""
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


class SimulationType(Enum):
    GAZEBO = "Gazebo"
    UNITY = "Unity"
    ISAAC_SIM = "IsaacSim"


class EnvironmentStatus(Enum):
    AVAILABLE = "Available"
    UNAVAILABLE = "Unavailable"
    NEEDS_SETUP = "NeedsSetup"


@dataclass
class SimulationEnvironment:
    """
    Represents a simulation environment used in the book's examples.
    """
    id: str
    name: str
    type: SimulationType
    description: str
    world_file: str
    robot_model: str
    launch_file: Optional[str] = None
    requirements: List[str] = None
    status: EnvironmentStatus = EnvironmentStatus.NEEDS_SETUP

    def __post_init__(self):
        """Validate the simulation environment after initialization."""
        if self.type not in [SimulationType.GAZEBO, SimulationType.UNITY, SimulationType.ISAAC_SIM]:
            raise ValueError("Type must be one of: Gazebo, Unity, IsaacSim")

        if not self.world_file:
            raise ValueError("worldFile must exist and be accessible")

    def validate_setup(self) -> bool:
        """Validate that the simulation environment is properly set up."""
        # Placeholder for validation logic
        return True

    def get_requirements(self) -> List[str]:
        """Get the system requirements for this environment."""
        if not self.requirements:
            return []
        return self.requirements
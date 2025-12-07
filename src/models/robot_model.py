"""
RobotModel model representing a humanoid robot model used in the book.
"""
from dataclasses import dataclass
from typing import List, Optional
from enum import Enum


class RobotModelType(Enum):
    PREBUILT = "Prebuilt"
    CUSTOM = "Custom"
    HYBRID = "Hybrid"


@dataclass
class JointDefinition:
    """Definition of a robot joint."""
    name: str
    type: str  # revolute, prismatic, fixed, etc.
    limits: Optional[dict] = None  # min, max, effort, velocity


@dataclass
class LinkDefinition:
    """Definition of a robot link."""
    name: str
    mass: float
    inertia: Optional[dict] = None  # ixx, ixy, ixz, iyy, iyz, izz


@dataclass
class SensorDefinition:
    """Definition of a robot sensor."""
    name: str
    type: str  # camera, lidar, imu, etc.
    position: Optional[List[float]] = None


@dataclass
class ActuatorDefinition:
    """Definition of a robot actuator."""
    name: str
    joint_name: str
    max_effort: float
    max_velocity: float


@dataclass
class RobotModel:
    """
    Represents a humanoid robot model used in the book.
    """
    id: str
    name: str
    type: RobotModelType
    urdf_path: Optional[str] = None
    sdf_path: Optional[str] = None
    joints: List[JointDefinition] = None
    links: List[LinkDefinition] = None
    sensors: List[SensorDefinition] = None
    actuators: List[ActuatorDefinition] = None
    capabilities: List[str] = None
    description: str = ""

    def __post_init__(self):
        """Validate the robot model after initialization."""
        if not self.urdf_path and not self.sdf_path:
            raise ValueError("Either urdfPath or sdfPath must be provided")

        if self.joints and self.links:
            # Check that joints and links form a valid kinematic chain
            # This is a simplified check - in reality, more complex validation would be needed
            pass

    def get_kinematic_chain(self) -> List[str]:
        """Get the kinematic chain of the robot."""
        if not self.joints:
            return []
        return [joint.name for joint in self.joints]

    def get_sensor_types(self) -> List[str]:
        """Get all sensor types in the robot."""
        if not self.sensors:
            return []
        return [sensor.type for sensor in self.sensors]
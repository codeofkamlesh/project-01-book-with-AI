"""
VLAPipeline model representing a Vision-Language-Action pipeline for humanoid control.
"""
from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from enum import Enum


class InputType(Enum):
    TEXT = "Text"
    VOICE = "Voice"
    GESTURE = "Gesture"


@dataclass
class ComponentDefinition:
    """Definition of a pipeline component."""
    name: str
    type: str  # perception, reasoning, action
    description: str
    requirements: List[str]


@dataclass
class ScenarioDefinition:
    """Definition of an example scenario."""
    name: str
    description: str
    expected_outcome: str


@dataclass
class ValidationCriteria:
    """Definition of validation criteria."""
    name: str
    description: str
    threshold: float
    unit: str


@dataclass
class VLAPipeline:
    """
    Represents a Vision-Language-Action pipeline for humanoid control.
    """
    id: str
    name: str
    description: str
    input_type: InputType
    perception_components: List[ComponentDefinition]
    reasoning_components: List[ComponentDefinition]
    action_components: List[ComponentDefinition]
    requirements: List[str]
    example_scenarios: List[ScenarioDefinition]
    validation_criteria: List[ValidationCriteria]

    def __post_init__(self):
        """Validate the VLA pipeline after initialization."""
        if not self.perception_components or not self.reasoning_components or not self.action_components:
            raise ValueError("Must have at least one component in each category (perception, reasoning, action)")

    def add_component(self, component: ComponentDefinition, component_type: str):
        """Add a component to the pipeline."""
        if component_type == "perception":
            self.perception_components.append(component)
        elif component_type == "reasoning":
            self.reasoning_components.append(component)
        elif component_type == "action":
            self.action_components.append(component)

    def execute_pipeline(self, input_data: Any) -> Dict[str, Any]:
        """Execute the VLA pipeline with the given input."""
        # Placeholder for pipeline execution logic
        result = {
            "success": True,
            "action_sequence": ["approach_object", "grasp_object", "lift_object"],
            "confidence": 0.87,
            "execution_time": 15.23
        }
        return result
"""
BookChapter model representing a single chapter in the book with its content, examples, and validation status.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional
from enum import Enum


class ChapterStatus(Enum):
    DRAFT = "Draft"
    REVIEW = "Review"
    VALIDATED = "Validated"
    PUBLISHED = "Published"


class ModelType(Enum):
    ROS2 = "ROS2"
    SIMULATION = "Simulation"
    ISAAC = "Isaac"
    VLA = "VLA"


@dataclass
class BookChapter:
    """
    Represents a single chapter in the book with its content, examples, and validation status.
    """
    id: str
    title: str
    model: ModelType
    content: str
    word_count: int
    diagrams: List[str]
    code_examples: List[str]
    success_criteria: List[str]
    status: ChapterStatus
    dependencies: List[str]
    created_at: datetime
    updated_at: datetime

    def __post_init__(self):
        """Validate the chapter after initialization."""
        if self.word_count > 1500:
            raise ValueError("Chapter word count must be ≤ 1500")

        if self.model not in [ModelType.ROS2, ModelType.SIMULATION, ModelType.ISAAC, ModelType.VLA]:
            raise ValueError("Model must be one of: ROS2, Simulation, Isaac, VLA")

    def validate_content(self) -> bool:
        """Validate the chapter content against success criteria."""
        # Placeholder for content validation logic
        return True

    def add_dependency(self, chapter_id: str):
        """Add a dependency to another chapter."""
        if chapter_id not in self.dependencies:
            self.dependencies.append(chapter_id)

    def update_status(self, new_status: ChapterStatus):
        """Update the chapter status."""
        self.status = new_status
        self.updated_at = datetime.now()
"""
Diagram model representing a diagram used in the book chapters.
"""
from dataclasses import dataclass
from typing import List
from enum import Enum


class DiagramType(Enum):
    CONCEPTUAL = "Conceptual"
    SYSTEM = "System"
    ARCHITECTURE = "Architecture"
    WORKFLOW = "Workflow"


class DiagramFormat(Enum):
    DRAWIO = "draw.io"
    PNG = "PNG"
    SVG = "SVG"


@dataclass
class Diagram:
    """
    Represents a diagram used in the book chapters.
    """
    id: str
    title: str
    type: DiagramType
    format: DiagramFormat
    file_path: str
    chapter_id: str
    description: str
    tags: List[str]

    def __post_init__(self):
        """Validate the diagram after initialization."""
        if self.format not in [DiagramFormat.DRAWIO, DiagramFormat.PNG, DiagramFormat.SVG]:
            raise ValueError("Format must be one of: draw.io, PNG, SVG")

        if not self.file_path:
            raise ValueError("filePath must exist and be accessible")

        if self.type not in [DiagramType.CONCEPTUAL, DiagramType.SYSTEM,
                            DiagramType.ARCHITECTURE, DiagramType.WORKFLOW]:
            raise ValueError("Type must be one of the defined values")

    def get_diagram_url(self) -> str:
        """Get the URL for accessing the diagram."""
        return f"diagrams/{self.file_path}"

    def validate_compatibility(self) -> bool:
        """Validate that the diagram is compatible with draw.io."""
        if self.format == DiagramFormat.DRAWIO:
            return True
        # For PNG and SVG, we assume they can be imported into draw.io
        return True
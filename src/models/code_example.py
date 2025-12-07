"""
CodeExample model representing a runnable code example within a chapter.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import List, Optional
from enum import Enum


class ExampleStatus(Enum):
    NOT_STARTED = "NotStarted"
    VALIDATED = "Validated"
    FAILED = "Failed"


class LanguageType(Enum):
    PYTHON = "Python"
    CPP = "C++"


@dataclass
class CodeExample:
    """
    Represents a runnable code example within a chapter.
    """
    id: str
    chapter_id: str
    title: str
    language: LanguageType
    code: str
    description: str
    requirements: List[str]
    validation_script: str
    status: ExampleStatus
    last_run: Optional[datetime] = None

    def __post_init__(self):
        """Validate the code example after initialization."""
        if self.language not in [LanguageType.PYTHON, LanguageType.CPP]:
            raise ValueError("Language must be supported (Python, C++)")

    def run_validation(self) -> bool:
        """Run the validation script for this example."""
        # Placeholder for validation logic
        # In a real implementation, this would execute the validation script
        # and return True if successful, False otherwise
        return True

    def update_status(self, new_status: ExampleStatus):
        """Update the example status."""
        self.status = new_status
        self.last_run = datetime.now()
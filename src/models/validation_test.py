"""
ValidationTest model representing a test used to validate book content and examples.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Optional
from enum import Enum


class TestType(Enum):
    CODE_EXECUTION = "CodeExecution"
    SIMULATION_LOAD = "SimulationLoad"
    BEHAVIOR_CHECK = "BehaviorCheck"
    DOCUMENTATION = "Documentation"


class TestStatus(Enum):
    PENDING = "Pending"
    PASSED = "Passed"
    FAILED = "Failed"
    SKIPPED = "Skipped"


@dataclass
class ValidationTest:
    """
    Represents a test used to validate book content and examples.
    """
    id: str
    name: str
    type: TestType
    target: str  # What the test validates (chapter, example, diagram, etc.)
    script: str  # Test script or command to execute
    expected_result: str
    status: TestStatus
    last_run: Optional[datetime] = None
    failure_reason: Optional[str] = None

    def __post_init__(self):
        """Validate the validation test after initialization."""
        if self.type not in [TestType.CODE_EXECUTION, TestType.SIMULATION_LOAD,
                            TestType.BEHAVIOR_CHECK, TestType.DOCUMENTATION]:
            raise ValueError("Type must be one of the defined values")

        if not self.script:
            raise ValueError("Script must be executable")

        if self.status not in [TestType.PENDING, TestType.PASSED, TestType.FAILED, TestType.SKIPPED]:
            raise ValueError("Status must be one of the defined values")

    def execute_test(self) -> bool:
        """Execute the validation test."""
        # Placeholder for test execution logic
        # In a real implementation, this would execute the test script
        # and return True if the test passes, False otherwise
        try:
            # Simulate test execution
            # This would actually run the script and check the result
            success = True  # Placeholder result
            self.status = TestStatus.PASSED if success else TestStatus.FAILED
            self.last_run = datetime.now()
            return success
        except Exception as e:
            self.status = TestStatus.FAILED
            self.failure_reason = str(e)
            self.last_run = datetime.now()
            return False

    def update_status(self, new_status: TestStatus, reason: Optional[str] = None):
        """Update the test status."""
        self.status = new_status
        self.last_run = datetime.now()
        if reason:
            self.failure_reason = reason
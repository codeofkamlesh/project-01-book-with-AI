# Data Model: AI/Spec-Driven Book Architecture

## Overview

This document defines the data models for the 4-model humanoid robotics book architecture (ROS2 → Simulation → Isaac → VLA). The models represent the core entities and their relationships that will be used throughout the book's content, examples, and validation systems.

## Core Entities

### 1. BookChapter
Represents a single chapter in the book with its content, examples, and validation status.

**Fields**:
- id: Unique identifier for the chapter
- title: Chapter title
- model: Associated model (ROS2, Simulation, Isaac, VLA)
- content: Markdown content of the chapter
- wordCount: Number of words in the chapter
- diagrams: List of associated diagrams
- codeExamples: List of code examples in the chapter
- successCriteria: List of success criteria to validate
- status: Draft, Review, Validated, Published
- dependencies: Other chapters this chapter depends on
- createdAt: Creation timestamp
- updatedAt: Last update timestamp

**Validation Rules**:
- wordCount must be ≤ 1500
- title must be unique within the book
- status must be one of the defined values
- model must be one of: ROS2, Simulation, Isaac, VLA

### 2. CodeExample
Represents a runnable code example within a chapter.

**Fields**:
- id: Unique identifier for the example
- chapterId: Reference to the parent chapter
- title: Brief description of the example
- language: Programming language (Python, C++, etc.)
- code: Source code content
- description: Explanation of what the code does
- requirements: System requirements for running the code
- validationScript: Script to validate the example
- status: NotStarted, Validated, Failed
- lastRun: Timestamp of last validation run

**Validation Rules**:
- language must be supported (Python, C++)
- code must be syntactically valid
- requirements must be clearly defined

### 3. SimulationEnvironment
Represents a simulation environment used in the book's examples.

**Fields**:
- id: Unique identifier for the environment
- name: Name of the environment
- type: Gazebo, Unity, IsaacSim
- description: Brief description of the environment
- worldFile: Path to the world/simulation file
- robotModel: Reference to the robot model used
- launchFile: ROS 2 launch file if applicable
- requirements: System requirements for the environment
- status: Available, Unavailable, NeedsSetup

**Validation Rules**:
- type must be one of: Gazebo, Unity, IsaacSim
- worldFile must exist and be accessible
- robotModel must be a valid reference

### 4. RobotModel
Represents a humanoid robot model used in the book.

**Fields**:
- id: Unique identifier for the robot model
- name: Name of the robot (e.g., "Atlas", "NAO", "Generic Humanoid")
- type: Prebuilt, Custom, Hybrid
- urdfPath: Path to URDF file if applicable
- sdfPath: Path to SDF file if applicable
- joints: List of joint definitions
- links: List of link definitions
- sensors: List of sensor configurations
- actuators: List of actuator configurations
- capabilities: List of robot capabilities
- description: Detailed description of the robot

**Validation Rules**:
- Either urdfPath or sdfPath must be provided
- joints and links must form a valid kinematic chain
- capabilities must be consistent with physical model

### 5. VLAPipeline
Represents a Vision-Language-Action pipeline for humanoid control.

**Fields**:
- id: Unique identifier for the pipeline
- name: Name of the pipeline
- description: Description of what the pipeline does
- inputType: Text, Voice, Gesture (or combination)
- perceptionComponents: List of perception modules
- reasoningComponents: List of reasoning modules
- actionComponents: List of action modules
- requirements: System requirements for the pipeline
- exampleScenarios: List of example scenarios
- validationCriteria: Criteria for validating pipeline success

**Validation Rules**:
- Must have at least one component in each category (perception, reasoning, action)
- Requirements must be clearly defined
- Example scenarios must be executable

### 6. Diagram
Represents a diagram used in the book chapters.

**Fields**:
- id: Unique identifier for the diagram
- title: Title of the diagram
- type: Conceptual, System, Architecture, Workflow
- format: draw.io, PNG, SVG
- filePath: Path to the diagram file
- chapterId: Reference to the chapter using this diagram
- description: Description of what the diagram illustrates
- tags: List of tags for categorization

**Validation Rules**:
- format must be one of: draw.io, PNG, SVG
- filePath must exist and be accessible
- type must be one of the defined values

### 7. ValidationTest
Represents a test used to validate book content and examples.

**Fields**:
- id: Unique identifier for the test
- name: Name of the test
- type: CodeExecution, SimulationLoad, BehaviorCheck, Documentation
- target: What the test validates (chapter, example, diagram, etc.)
- script: Test script or command to execute
- expectedResult: Expected outcome of the test
- status: Pending, Passed, Failed, Skipped
- lastRun: Timestamp of last test execution
- failureReason: Reason for failure if status is Failed

**Validation Rules**:
- type must be one of the defined values
- script must be executable
- status must be one of the defined values

## Relationships

### BookChapter Relationships
- BookChapter → CodeExample (one-to-many): A chapter contains multiple code examples
- BookChapter → Diagram (one-to-many): A chapter contains multiple diagrams
- BookChapter → ValidationTest (one-to-many): A chapter has multiple validation tests
- BookChapter → BookChapter (many-to-many): Chapters can have dependencies on other chapters

### CodeExample Relationships
- CodeExample → BookChapter (many-to-one): A code example belongs to one chapter
- CodeExample → ValidationTest (one-to-many): A code example has multiple validation tests

### SimulationEnvironment Relationships
- SimulationEnvironment → RobotModel (many-to-one): A simulation environment uses one robot model
- SimulationEnvironment → BookChapter (many-to-many): A simulation environment can be used in multiple chapters
- SimulationEnvironment → CodeExample (one-to-many): A simulation environment can run multiple code examples

### RobotModel Relationships
- RobotModel → SimulationEnvironment (one-to-many): A robot model can be used in multiple simulation environments
- RobotModel → CodeExample (one-to-many): A robot model can be used in multiple code examples

### VLAPipeline Relationships
- VLAPipeline → BookChapter (many-to-one): A VLA pipeline belongs to one chapter (or is used in one primary chapter)
- VLAPipeline → ValidationTest (one-to-many): A VLA pipeline has multiple validation tests

### Diagram Relationships
- Diagram → BookChapter (many-to-one): A diagram belongs to one chapter

### ValidationTest Relationships
- ValidationTest → BookChapter (many-to-one): A validation test validates one chapter
- ValidationTest → CodeExample (many-to-one): A validation test validates one code example

## State Transitions

### BookChapter States
- Draft → Review: Content is complete and ready for review
- Review → Validated: Content has been reviewed and validated
- Validated → Published: Content is ready for publication

### CodeExample States
- NotStarted → Validated: Code example has been tested and validated
- NotStarted → Failed: Code example failed validation
- Validated → Failed: Code example was previously validated but now fails (regression)
- Failed → Validated: Code example was fixed and re-validated

### ValidationTest States
- Pending → Passed: Test executed successfully
- Pending → Failed: Test executed but did not meet expectations
- Passed → Failed: Previously passing test now fails (regression)
- Failed → Passed: Previously failing test now passes

## Constraints and Business Rules

1. **Chapter Length Constraint**: Each BookChapter must have a wordCount ≤ 1500
2. **Model Sequence Constraint**: Chapters must follow the sequence ROS2 → Simulation → Isaac → VLA where dependencies exist
3. **Validation Requirement**: All CodeExamples must pass validation before a BookChapter can be marked as Validated
4. **Diagram Compatibility**: All Diagrams must be in draw.io format to ensure compatibility
5. **Reproducibility**: All CodeExamples and SimulationEnvironments must run with standard installations
6. **Documentation Accuracy**: All content must be validated against official documentation
7. **Cross-Model Integration**: Examples should demonstrate integration between different models where appropriate

## Indexes and Performance Considerations

1. **Chapter Index**: By model type for quick navigation
2. **Example Index**: By language and complexity for targeted learning
3. **Validation Index**: By status for quick identification of issues
4. **Dependency Index**: For tracking chapter dependencies and validation order
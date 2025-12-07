<!-- SYNC IMPACT REPORT:
Version change: N/A → 1.0.0
Added sections: Core Principles (6), Project Constraints, Development Workflow, Governance
Removed sections: None (first version)
Templates requiring updates: N/A (first version)
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation using Docusaurus, Spec-Kit Plus, and Claude Code Constitution

## Core Principles

### Accuracy and correctness
All technical explanations must be validated through official documentation or authoritative sources. Verified technical content is paramount.

### Clarity for learners
Content must be designed for beginner–intermediate developers. Writing style should be concise, structured, and instructional.

### Consistency with Spec-Kit Plus spec-driven workflow
All development follows the Spec-Kit Plus methodology. Practicality: hands-on, example-driven explanations.

### Code quality and reproducibility
Code examples must be tested and reproducible. Format: Docusaurus-ready Markdown with proper front-matter.

### No ambiguity
Each chapter must have clear objectives and outcomes. No ambiguity in instructions or explanations.

### Deployment reliability
Success criteria: All chapters pass linting, build, and preview checks. Book fully deploys on GitHub Pages without error.

## Project Constraints
Book length: 8–12 core chapters. Each chapter ≤ 1,500 words. Must include diagrams and workflows where helpful. Deployment target: GitHub Pages via Docusaurus build pipeline.

## Development Workflow
All outputs strictly follow the user intent. Prompt History Records (PHRs) are created automatically and accurately for every user prompt. Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions. All changes are small, testable, and reference code precisely.

## Governance
All changes must comply with the core principles. Constitution supersedes all other practices. Amendments require documentation and approval. All outputs must pass validation checks before completion.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07

<!-- SYNC IMPACT REPORT:
Version change: 1.0.2 → 1.0.3
Added sections: AI-Native Book Capabilities
Modified sections: AI-Augmented Book Intelligence (expanded with specific constraints)
Templates requiring updates: N/A
Follow-up TODOs: None
-->
# AI-Native Hackathon Project — Full Spec-Driven System Constitution

## Core Principles

### Primary-source accuracy
All claims and references must be validated directly from authoritative origins.

### AI-native workflow
All deliverables must follow SpecKit+, Context7 MCP architecture, and Docusaurus UI.

### Clarity & readability
Content must be written for general technical audiences (CS, AI, and software engineering background).

### Structural rigor
All modules must follow hierarchical spec structure (Module → Chapters → Subsections).

### Traceable reproducibility
Every claim, diagram, and design decision must be traceable to its corresponding spec or model.

## Academic Writing Standards

### Factual traceability
Every factual statement must have a source.

### Citation style
APA (7th edition)

### Source quality
At least 50% peer-reviewed journal articles

### Plagiarism policy
0% tolerance; all content must be fully original or properly quoted

### Readability requirement
Flesch-Kincaid Grade 10–12

## Document Constraints
- Total content per major module: 5,000–7,000 words
- Minimum references per major module: 15 sources
- Output format: PDF with embedded citations + Markdown source for Docusaurus
- Visual content: All diagrams must be created through Draw.io, exported, and referenced in the UI

## System Constraints
- Frontend rendered using Docusaurus with collapsible sidebar for chapters
- Full Context7 MCP Server integration for specification → code → validation flow
- All modules must be organized using the structure:
  - Module 1: Constitution Phase
  - Module 2: Specification Phase
  - Module 3: Modeling Phase
  - Module 4: Implementation Phase
  - Module 5: Reflection & Evaluation Phase

## Development Workflow
All outputs strictly follow the user intent. Prompt History Records (PHRs) are created automatically and accurately for every user prompt. Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions. All changes are small, testable, and reference code precisely.

## Governance
All changes must comply with the core principles. Constitution supersedes all other practices. Amendments require documentation and approval. All outputs must pass validation checks before completion.

## Success Criteria
- All project components comply with constitution rules
- All claims are source-validated
- All modules meet word count and reference requirements
- Deployment succeeds on GitHub Pages via Docusaurus
- All architectural decisions properly documented in ADRs

## AI-Augmented Book Intelligence
The system MAY include AI-driven features that enhance the educational experience while maintaining all constitutional principles:

### AI Features Permitted
- Retrieval-Augmented Generation (RAG) strictly grounded in book content with provenance tracking
- Authentication-backed personalization that adapts content complexity based on user profile
- Language translation as a non-destructive view layer that preserves original content
- Reusable intelligence via spec-defined subagents for educational content generation

### AI Constraints
- All AI outputs must be traceable to source documents in the book
- No hallucination is permitted - AI must only use provided context
- All AI-generated content must be clearly labeled as AI-generated
- Original content remains unchanged - AI provides enhancement layers only

## AI-Native Book Capabilities
The system MUST implement AI-native book features with the following mandatory requirements:

### RAG Content Constraint
- RAG must answer only from book corpus, never from external sources
- All responses must include provenance information citing specific book sections
- No knowledge beyond the book content is permitted in RAG responses

### Spec-Traceability Requirement
- All AI features must be spec-traceable from requirement to implementation
- Each AI capability must map to specific sections in the project specifications
- No "vibe coding" allowed - all AI features must follow documented specifications

### Context7 MCP Validation
- Context7 MCP is mandatory for AI reasoning validation
- All AI reasoning steps must be validated through the Context7 MCP architecture
- AI outputs must pass MCP validation before being presented to users

### AI Feature Requirements
- Integrated RAG chatbot for book content questions with proper provenance
- Authentication & user profiling for personalized learning experiences
- Personalized content rendering based on user background and preferences
- Urdu translation as a non-destructive toggle preserving original content
- Reusable intelligence (subagents + skills) for educational content generation

**Version**: 1.0.3 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15

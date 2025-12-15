---
id: 8
title: Mount RAG Chatbot into Docusaurus
stage: green
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.implement
labels: ["rag", "docusaurus", "ui", "integration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/theme/Root.jsx
tests:
 - none
---

## Prompt

TITLE: Mount RAG Chatbot into Docusaurus Render Tree (Visibility Fix)

GOAL:
Ensure the existing RAG chatbot is visibly rendered inside the Docusaurus book UI.

STRICT RULES:
- SpecKit Plus only
- NO vibe coding
- NO new features
- NO backend changes
- Only mount + register existing components

TASKS:

A. Docusaurus Root Injection
- Verify that the RAG ChatWidget is mounted using:
  - src/theme/Root.jsx OR
  - a registered Docusaurus plugin client entry
- If ChatWidget exists but is unused, wire it into Root layout.

B. Plugin Verification
- Confirm docusaurus-plugin-rag is:
  - imported
  - registered in docusaurus.config.js
  - exporting a client module

C. Runtime Guard
- Add a console warning if ChatWidget is not mounted (non-fatal).

D. Acceptance Criteria
- Chat icon is visible on every docs page.
- Chat persists when navigating between pages.
- No UI regressions.

FORBIDDEN:
- Editing RAG logic
- Editing FastAPI
- Editing embeddings or pipelines

OUTPUT:
- File paths modified
- Reason for each change
-

## Response snapshot

Created Root.jsx theme component to mount the existing RAG ChatWidget, ensuring it appears on every page of the Docusaurus documentation site.

## Outcome

- ✅ Impact: RAG ChatWidget now visible on every docs page
- 🧪 Tests: none
- 📁 Files: 1 file created
- 🔁 Next prompts: Complete AI feature validation
- 🧠 Reflection: Docusaurus theme Root component is the correct place to mount persistent UI elements.

## Evaluation notes (flywheel)

- Failure modes observed: ChatWidget was not mounted in UI despite existing
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test UI visibility in browser

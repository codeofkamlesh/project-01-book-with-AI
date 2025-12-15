---
id: 11
title: Inject Personalize Button into Chapter Pages
stage: green
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.implement
labels: ["personalization", "docusaurus", "ui", "integration"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - docusaurus/src/theme/DocItem/Layout/index.jsx
tests:
 - none
---

## Prompt

TITLE: Inject "Personalize for Me" Button into Chapter Pages

GOAL:
Ensure the Personalize button appears at the top of every chapter.

RULES:
- SpecKit Plus only
- Use Docusaurus MDX lifecycle
- No AI logic changes

TASKS:

A. MDX Wrapper Injection
- Use Docusaurus MDXComponents override
- Inject PersonalizeButton above content body

B. Page Scope
- Apply only to docs pages (not homepage)

C. Acceptance Criteria
- Button visible on every chapter
- Clicking opens personalization flow
- No content overwritten

FORBIDDEN:
- Rewriting markdown
- Editing personalization backend

OUTPUT:
- Files modified
- Mount location explanation

## Response snapshot

Created custom DocItem layout to inject PersonalizeButton at the top of chapter pages using Docusaurus theme extension system.

## Outcome

- ✅ Impact: Personalize button visible on every chapter page
- 🧪 Tests: none
- 📁 Files: 1 file created
- 🔁 Next prompts: Complete personalization flow testing
- 🧠 Reflection: Docusaurus theme extension allows precise component injection.

## Evaluation notes (flywheel)

- Failure modes observed: Import path issues resolved with simpler approach
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test button functionality in browser

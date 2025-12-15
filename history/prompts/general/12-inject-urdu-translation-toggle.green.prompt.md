---
id: 12
title: Inject Urdu Translation Toggle into Chapters
stage: green
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.implement
labels: ["translation", "docusaurus", "ui", "integration"]
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

TITLE: Inject Urdu Translation Toggle into Chapters

GOAL:
Ensure "Translate to Urdu" button is visible and toggleable.

RULES:
- SpecKit Plus only
- No translation logic changes

TASKS:

A. Shared Control Bar
- Mount UrduTranslationButton next to Personalize button
- Use same MDX injection point

B. Toggle Safety
- Ensure English content is not overwritten

C. Acceptance Criteria
- Toggle visible
- Urdu text renders
- Toggle off restores English

FORBIDDEN:
- Overwriting markdown
- Changing translation API

OUTPUT:
- Verified toggle behavior

## Response snapshot

Updated DocItem layout to include both PersonalizeButton and UrduTranslationButton in a shared control bar at the top of chapter pages.

## Outcome

- ✅ Impact: Urdu translation toggle visible next to personalize button
- 🧪 Tests: none
- 📁 Files: 1 file modified
- 🔁 Next prompts: Complete translation flow testing
- 🧠 Reflection: Combined controls in shared control bar for better UX.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test both buttons in browser

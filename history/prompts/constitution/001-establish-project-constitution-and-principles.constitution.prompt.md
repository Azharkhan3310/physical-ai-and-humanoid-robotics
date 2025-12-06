---
id: 001
title: Establish Project Constitution and Principles
stage: constitution
date: 2025-12-06
surface: agent
model: gemini
feature: none
branch: master
user: {{USER}}
command: /sp.constitution
labels: ["constitution", "governance", "principles"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .specify/templates/plan-template.md
tests:

---

## Prompt

/sp.constitution

Project: AI/Spec-Driven Book Creation Using Docusaurus, Spec-Kit Plus, and Claude Code

Core principles:
- Spec-driven authorship: All chapters, sections, and decisions must trace back to written specs.
- Technical accuracy: Content must remain factually correct regarding tooling (Docusaurus, GitHub Pages, Spec-Kit Plus, Claude Code, AI workflows).
- Clarity for beginner–intermediate developers: Writing should be understandable to readers with basic web development knowledge.
- Modularity & Reusability: Book structure must follow clean modular docs, each chapter self-contained.
- AI-assisted workflow integrity: All generated content must comply with Spec-Kit Plus methodology (constitutions → specs → tasks).

Key standards:
- Structure:
  - Book must follow a well-organized Docusaurus structure (`docs/`, `sidebar`, `mdx` files).
  - Each chapter should align with a corresponding spec file.
- Tooling Standardization:
  - All instructions must be verifiable using real commands for Docusaurus, GitHub Pages deployment, and GitHub Actions.
  - Any code examples must be tested or syntactically valid.
- Writing Quality:
  - Tone: educational, beginner-friendly, and practical.
  - Style: clean, concise technical writing.
  - Flesch Reading Score: Grade 9–12.
- AI citation requirement:
  - Any claims about tools, features, commands, or frameworks must be based on primary documentation sources.
  - Links to docs preferred (e.g., Docusaurus official docs, GitHub documentation).
- Zero fluff:
  - No filler content or speculative information.
  - Every paragraph must add practical instructional value.

Constraints:
- Book Length: 8–12 chapters
- Total Word Count: 12,000–18,000 words
- Output Format:
  - Markdown/MDX compatible with Docusaurus
  - GitHub Pages deployable
- AI Workflow Requirements:
  - All content must be generated through Spec-Kit Plus workflow (constitution → specs → tasks).
- Version Control:
  - Commit messages must follow Conventional Commits format.
- No plagiarism:
  - 0% tolerance for copied text from external sources.
  - Summaries allowed but must be rewritten in original phrasing.

Success criteria:
- Book builds successfully in Docusaurus with no broken pages or sidebar errors.
- Deploys successfully to GitHub Pages using GitHub Actions workflow.
- All chapters trace back to specs generated under Spec-Kit Plus.
- Accurate and complete documentation on:
  - Setting up Docusaurus
  - Structuring the book
  - Integrating Spec-Kit Plus
  - Using Claude Code for writing
  - Deploying to GitHub Pages
- Beginner-friendly: readers should be able to reproduce the entire setup step by step.
- No factual mistakes about tools, commands, workflows, or configurations.
- Coherent narrative across chapters with consistent formatting.

## Response snapshot

The constitution at `.specify/memory/constitution.md` was updated to version 1.0.0, incorporating the user's provided principles, standards, constraints, and success criteria. A Sync Impact Report was added as a comment in the file. The `plan-template.md` was updated to reflect the new principles in its 'Constitution Check' section.

## Outcome

- ✅ Impact: High. Established the foundational governance document for the project.
- 🧪 Tests: None.
- 📁 Files: 2 files modified.
- 🔁 Next prompts: n/a
- 🧠 Reflection: The process was smooth, but the initial prompt contained conflicting instructions regarding script paths and file locations, which required adaptation.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a

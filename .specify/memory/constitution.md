<!--
---
Sync Impact Report:
- Version change: none -> 1.0.0
- List of modified principles:
  - New: Spec-Driven Authorship
  - New: Technical Accuracy
  - New: Clarity for Beginner–Intermediate Developers
  - New: Modularity & Reusability
  - New: AI-Assisted Workflow Integrity
  - New: Standardized Structure
  - New: Tooling Standardization
  - New: High-Quality Writing
  - New: Primary Source Citation
  - New: Zero Fluff
- Added sections:
  - Core Principles
  - Project Constraints
  - Success Criteria
  - Governance
- Removed sections:
  - n/a (placeholders filled)
- Templates requiring updates:
  - ⚠ pending: .specify/templates/plan-template.md
  - ⚠ pending: .specify/templates/spec-template.md
  - ⚠ pending: .specify/templates/tasks-template.md
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Determine original adoption date.
---
-->
# AI/Spec-Driven Book Creation Using Docusaurus, Spec-Kit Plus, and Claude Code Constitution

## Core Principles

### I. Spec-Driven Authorship
All chapters, sections, and decisions must trace back to written specs.

### II. Technical Accuracy
Content must remain factually correct regarding tooling (Docusaurus, GitHub Pages, Spec-Kit Plus, Claude Code, AI workflows).

### III. Clarity for Beginner–Intermediate Developers
Writing should be understandable to readers with basic web development knowledge.

### IV. Modularity & Reusability
Book structure must follow clean modular docs, each chapter self-contained.

### V. AI-Assisted Workflow Integrity
All generated content must comply with Spec-Kit Plus methodology (constitutions → specs → tasks).

### VI. Standardized Structure
- Book must follow a well-organized Docusaurus structure (`docs/`, `sidebar`, `mdx` files).
- Each chapter should align with a corresponding spec file.

### VII. Tooling Standardization
- All instructions must be verifiable using real commands for Docusaurus, GitHub Pages deployment, and GitHub Actions.
- Any code examples must be tested or syntactically valid.

### VIII. High-Quality Writing
- Tone: educational, beginner-friendly, and practical.
- Style: clean, concise technical writing.
- Flesch Reading Score: Grade 9–12.

### IX. Primary Source Citation
- Any claims about tools, features, commands, or frameworks must be based on primary documentation sources.
- Links to docs preferred (e.g., Docusaurus official docs, GitHub documentation).

### X. Zero Fluff
- No filler content or speculative information.
- Every paragraph must add practical instructional value.

## Project Constraints

- **Book Length**: 8–12 chapters
- **Total Word Count**: 12,000–18,000 words
- **Output Format**:
  - Markdown/MDX compatible with Docusaurus
  - GitHub Pages deployable
- **AI Workflow Requirements**:
  - All content must be generated through Spec-Kit Plus workflow (constitution → specs → tasks).
- **Version Control**:
  - Commit messages must follow Conventional Commits format.
- **No plagiarism**:
  - 0% tolerance for copied text from external sources.
  - Summaries allowed but must be rewritten in original phrasing.

## Success Criteria

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

## Governance

This Constitution is the single source of truth for project principles and standards. All development, content, and contributions MUST adhere to it. Amendments require a documented proposal, review, and approval from the project owner.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): Determine original adoption date. | **Last Amended**: 2025-12-06
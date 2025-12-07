# Tasks: Humanoid Robotics Book Feature

**Input**: Design documents from `/specs/001-humanoid-robotics-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit request for TDD approach, so tests will not be generated as separate tasks unless naturally integrated into verification steps.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

## Path Conventions

-   **Web app**: `docusaurus/` at repository root, then `docs/`, `src/components/`, etc.
-   Paths shown below assume this structure.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize the Docusaurus project and essential configurations.

- [x] T001 Initialize a Docusaurus project if not already present in `docusaurus/`
- [x] T002 Configure `docusaurus.config.ts` for basic site metadata (title, tagline, URL, favicon) in `docusaurus/docusaurus.config.ts`
- [x] T003 [P] Configure basic CSS styling in `docusaurus/src/css/custom.css`
- [x] T004 Define initial sidebar structure for the book in `docusaurus/sidebars.ts`
- [x] T005 Add an introductory documentation page for the book (e.g., `docusaurus/docs/00-introduction.mdx`)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Docusaurus infrastructure that MUST be complete before any user story content can be added or displayed.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

-   [ ] T006 Ensure Docusaurus site builds successfully in development mode (`npm start` in `docusaurus/`)
-   [ ] T007 Ensure Docusaurus site builds successfully for production (`npm run build` in `docusaurus/`)
-   [ ] T008 Configure GitHub Pages deployment workflow (GitHub Actions) in `.github/workflows/deploy.yml`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access Book Documentation (Priority: P1) 🎯 MVP

**Goal**: A user can access, navigate, and search the comprehensive book documentation.

**Independent Test**: Navigate to various book sections, verify content display, and test search functionality.

### Implementation for User Story 1

-   [ ] T009 [P] [US1] Create example chapter directory (`docusaurus/docs/01-example-chapter/`)
-   [ ] T010 [US1] Add initial content page within example chapter (`docusaurus/docs/01-example-chapter/01-first-page.md`)
-   [ ] T011 [US1] Update `docusaurus/sidebars.ts` to include the new example chapter and page
-   [ ] T012 [P] [US1] Implement basic search functionality using Docusaurus's built-in search or a lightweight plugin in `docusaurus/docusaurus.config.ts`
-   [ ] T013 [US1] Verify content rendering and hierarchical navigation in `docusaurus/`
-   [ ] T014 [P] [US1] Ensure documentation pages are responsive across different screen sizes by adjusting CSS in `docusaurus/src/css/custom.css`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Explore Code Examples (Priority: P2)

**Goal**: A user can easily find, view with highlighting, and potentially download/copy code examples directly related to the book content.

**Independent Test**: Verify code examples are displayed correctly with syntax highlighting, and that copy/download mechanisms (if implemented) function.

### Implementation for User Story 2

-   [ ] T015 [P] [US2] Create an `examples/` directory for code snippets at the repository root
-   [ ] T016 [US2] Add a sample code example file to `examples/ch01/hello_world.py`
-   [ ] T017 [US2] Embed the sample code example within an existing documentation page, ensuring syntax highlighting in `docusaurus/docs/01-example-chapter/01-first-page.md`
-   [ ] T018 [P] [US2] Explore Docusaurus plugins for code block enhancements (e.g., copy-to-clipboard button) and configure in `docusaurus/docusaurus.config.ts`
-   [ ] T019 [US2] Implement or integrate a mechanism for users to download code example files from documentation pages (if not covered by copy/paste or direct linking) in `docusaurus/src/components/CodeDownloadLink.tsx`
-   [ ] T020 [US2] Add versioning information/notes to a relevant documentation page for the sample code example in `docusaurus/docs/01-example-chapter/01-first-page.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and overall site quality.

-   [ ] T021 [P] Implement a custom 404 page for missing content in `docusaurus/src/pages/404.js` (or `404.tsx`)
-   [ ] T022 [P] Add a "Last updated" footer to documentation pages (Docusaurus feature) in `docusaurus/docusaurus.config.ts`
-   [ ] T023 Code cleanup and refactoring for Docusaurus components and configuration files
-   [ ] T024 Verify all internal links are functional within the Docusaurus site
-   [ ] T025 Run `quickstart.md` validation: Follow all steps in `quickstart.md` to ensure they are accurate and work as described.

---

## Phase 6: Initial Book Module Content

**Purpose**: Create placeholder directories and introductory pages for additional book modules identified in the `00-introduction.mdx` overview.

-   [ ] T026 [P] Create directory for Module 4: The AI-Robot Brain (NVIDIA Isaac Sim) in `docusaurus/docs/04-module-ai-robot-brain/`
-   [ ] T027 [US_MOD4] Add introductory page for Module 4 in `docusaurus/docs/04-module-ai-robot-brain/01-introduction.mdx`
-   [ ] T028 [P] Create directory for Module 5: Vision-Language-Action (VLA) Systems in `docusaurus/docs/05-module-vla-systems/`
-   [ ] T029 [US_MOD5] Add introductory page for Module 5 in `docusaurus/docs/05-module-vla-systems/01-introduction.mdx`
-   [ ] T030 [P] Create directory for Module 6: Deploying Sim-to-Real Workflows in `docusaurus/docs/06-module-sim-to-real/`
-   [ ] T031 [US_MOD6] Add introductory page for Module 6 in `docusaurus/docs/06-module-sim-to-real/01-introduction.mdx`
-   [ ] T032 Update `docusaurus/sidebars.ts` to include Module 4, Module 5, and Module 6 in the `bookSidebar`

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
-   **User Stories (Phase 3+)**: All depend on Foundational phase completion
    -   User stories can then proceed in parallel (if staffed)
    -   Or sequentially in priority order (P1 → P2 → P3)
-   **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
-   **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable

### Within Each User Story

-   Models/Entities (if applicable) before Services
-   Services before UI/Page components
-   Core implementation before integration
-   Story complete before moving to next priority

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel
-   All Foundational tasks (none marked [P] in this case, but could be)
-   Once Foundational phase completes, User Story 1 and User Story 2 can ideally be worked on in parallel by different team members (though US1 is higher priority).
-   Within User Story 1, T009, T012, T014 can be parallel.
-   Within User Story 2, T015, T018 can be parallel.
-   Polish tasks marked [P] can run in parallel.

---

## Parallel Example: User Story 1

```bash
# Example of parallel tasks for User Story 1:
# Create an example chapter directory and configure search simultaneously.
Task: "T009 [P] [US1] Create example chapter directory (docusaurus/docs/01-example-chapter/)"
Task: "T012 [P] [US1] Implement basic search functionality using Docusaurus's built-in search or a lightweight plugin in docusaurus/docusaurus.config.ts"
```

## Implementation Strategy

### MVP First (User Story 1 Only)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1
4.  **STOP and VALIDATE**: Test User Story 1 independently
5.  Deploy/demo if ready

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready
2.  Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3.  Add User Story 2 → Test independently → Deploy/Demo
4.  Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together
2.  Once Foundational is done:
    -   Developer A: User Story 1
    -   Developer B: User Story 2
3.  Stories complete and integrate independently

---

## Notes

-   [P] tasks = different files, no dependencies
-   [Story] label maps task to specific user story for traceability
-   Each user story should be independently completable and testable
-   Commit after each task or logical group
-   Stop at any checkpoint to validate story independently
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

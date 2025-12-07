# Data Model: Humanoid Robotics Book Feature

**Feature**: Humanoid Robotics Book Feature
**Date**: 2025-12-07

## Entities

### Documentation Page

**Description**: Represents a single unit of content within the book, intended to be displayed as a navigable page in the Docusaurus documentation site.

**Attributes**:
- **id**: Unique identifier for the page (e.g., file path relative to docs directory, or a generated slug).
- **title**: The human-readable title of the page.
- **content**: The main body of the page, including text, embedded images, and references to code snippets. Format is Markdown/MDX.
- **chapter_id**: Reference to the chapter this page belongs to, establishing hierarchical structure.
- **tags**: Optional keywords for categorization and search.
- **last_modified_date**: Timestamp of the last update to the page content.
- **code_examples_referenced**: A list of `Code Example` IDs embedded within or linked from this page.

**Relationships**:
- Belongs to a Chapter (implicit in Docusaurus structure via directory hierarchy/sidebar).
- Contains/references multiple `Code Example` entities.

### Code Example

**Description**: A runnable snippet or file that demonstrates a specific robotics concept, directly associated with `Documentation Page` content.

**Attributes**:
- **id**: Unique identifier for the code example (e.g., file path, or a generated hash).
- **name**: A short, descriptive name for the example.
- **language**: The programming language of the example (e.g., Python, C++, ROS DSL).
- **code_body**: The actual source code of the example.
- **file_path**: The relative path to the source file within the project's examples directory (if applicable).
- **version_info**: Information about software dependencies and versions required for the example (e.g., ROS2 version, Python library versions).
- **description**: A brief explanation of what the code example does and demonstrates.

**Relationships**:
- Can be referenced by one or more `Documentation Page` entities.
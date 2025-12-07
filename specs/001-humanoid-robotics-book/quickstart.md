# Quickstart Guide: Humanoid Robotics Book Feature

**Feature**: Humanoid Robotics Book Feature
**Date**: 2025-12-07

This quickstart guide provides instructions for setting up and working with the Docusaurus-based documentation site for the Humanoid Robotics Book.

## Prerequisites

Before you begin, ensure you have the following installed:

-   Node.js (LTS version recommended)
-   npm (Node Package Manager, usually comes with Node.js)
-   Git

## 1. Clone the Repository

First, clone the project repository to your local machine:

```bash
git clone [REPOSITORY_URL]
cd physical-ai-and-humanoid-robotics
```
*(Replace `[REPOSITORY_URL]` with the actual URL of your project repository)*

## 2. Install Dependencies

Navigate to the `docusaurus` directory and install the necessary Node.js dependencies:

```bash
cd docusaurus
npm install
```

## 3. Start the Local Development Server

Once the dependencies are installed, you can start the Docusaurus development server to view the book locally:

```bash
npm start
```

This command will open a new browser window/tab at `http://localhost:3000` (or another port if 3000 is in use), displaying the documentation site. Any changes you make to the Markdown/MDX files will automatically hot-reload in your browser.

## 4. Building for Production

To generate a static production build of the documentation site:

```bash
npm run build
```

This command will compile the site into static HTML, CSS, and JavaScript files in the `build` directory, ready for deployment.

## 5. Adding New Content

To add new chapters or pages:

1.  Create a new Markdown (`.md`) or MDX (`.mdx`) file within the `docusaurus/docs/` directory, following the existing naming conventions (e.g., `01-chapter-name/01-page-name.md`).
2.  Update `docusaurus/sidebars.ts` to include your new content in the navigation structure.

## 6. Deploying to GitHub Pages

Deployment is typically handled via GitHub Actions. Push your changes to the `master` or designated deployment branch, and the CI/CD pipeline will automatically build and deploy the site to GitHub Pages.

---
**Note**: This quickstart guide assumes a standard Docusaurus setup and deployment to GitHub Pages. Refer to the official Docusaurus documentation for advanced configurations or troubleshooting.
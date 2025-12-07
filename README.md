# Physical AI & Humanoid Robotics Textbook

This directory contains the Docusaurus-based textbook for "Physical AI & Humanoid Robotics".

## Prerequisites

- Node.js 18 or higher
- npm or yarn package manager

## Installation

```bash
cd book
npm install
```

## Local Development

```bash
cd book
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
cd book
npm run build
```

This command generates static content into the `build` directory that can be served using any static contents hosting service.

## Deployment

The site is configured to deploy to GitHub Pages. When pushing to the main branch, it will automatically build and deploy if using GitHub Actions.

## Directory Structure

- `docs/` - Contains all textbook content in Markdown/MDX format
- `src/` - Contains custom React components and CSS
- `static/` - Contains static assets like images
- `docusaurus.config.js` - Main configuration file
- `sidebars.js` - Navigation sidebar configuration

## Adding New Content

To create a new document:

1. Create a new Markdown file in the appropriate directory under `docs/`
2. Add the file to the sidebar configuration in `sidebars.js`
3. Use the frontmatter format to specify metadata:

```markdown
---
sidebar_position: 1
---

# Document Title

Content here...
```

## Custom Components

This textbook includes several custom components for educational content:

- `LearningObjectives` - For displaying chapter learning objectives
- `ExerciseBox` - For exercises, examples, and assessments
- `Admonition` - For notes, tips, and warnings
- Custom CSS for textbook styling

## Tech Stack

- [Docusaurus v3](https://docusaurus.io/) - Static site generator
- [React](https://reactjs.org/) - Component library
- [MDX](https://mdxjs.com/) - JSX in Markdown
- [Prism React Renderer](https://github.com/FormidableLabs/prism-react-renderer) - Syntax highlighting
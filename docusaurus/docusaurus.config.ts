import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Book',
  tagline: 'A Practical Guide to Building Autonomous Humanoid Robots from Simulation to Reality',
  favicon: 'img/favicon.ico',

  future: {
    v4: true,
  },

  url: 'https://physical-ai-robotics.github.io',
  baseUrl: '/physical-ai-and-humanoid-robotics/',

  organizationName: 'physical-ai-robotics',
  projectName: 'physical-ai-and-humanoid-robotics',

  onBrokenLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/physical-ai-robotics/physical-ai-and-humanoid-robotics/tree/main/docusaurus/',
        },
        blog: false, // Explicitly disable blog
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'examples',
        path: '../examples', // Path relative to docusaurus directory
        routeBasePath: 'examples', // URL base path for these docs
        sidebarPath: false, // Don't create a sidebar for examples
        include: ['**/*.md', '**/*.mdx'], // Include all markdown files
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics Book',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Book Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'bookSidebar', // Changed from tutorialSidebar
          position: 'left',
          label: 'Book', // Changed from Tutorial
        },
        {
          href: 'https://github.com/physical-ai-robotics/physical-ai-and-humanoid-robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Start Reading',
              to: '/docs/intro', // Will be intro.mdx in 00-introduction.mdx
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub Discussions',
              href: 'https://github.com/physical-ai-robotics/physical-ai-and-humanoid-robotics/discussions',
            },
            // Removed Discord
            // Removed X
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub Repository',
              href: 'https://github.com/physical-ai-robotics/physical-ai-and-humanoid-robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;

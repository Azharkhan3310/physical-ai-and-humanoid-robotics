module.exports = {
  root: true,
  env: {
    browser: true,
    es2021: true,
    node: true,
  },
  extends: [
    'eslint:recommended',
    'plugin:@typescript-eslint/recommended',
    'plugin:react/recommended',
    'plugin:react-hooks/recommended',
    'plugin:jsx-a11y/recommended',
    'plugin:prettier/recommended', // Make sure this is always the last configuration in the extends array.
  ],
  parser: '@typescript-eslint/parser',
  parserOptions: {
    ecmaFeatures: {
      jsx: true,
    },
    ecmaVersion: 12,
    sourceType: 'module',
  },
  plugins: [
    '@typescript-eslint',
    'react',
    'react-hooks',
    'jsx-a11y',
    'prettier',
  ],
  rules: {
    'prettier/prettier': 'error',
    // Example: Docusaurus often requires specific global variables
    'no-undef': 'off', // Docusaurus defines globals
    'react/react-in-jsx-scope': 'off', // Next.js and new React versions don't need this
    '@typescript-eslint/explicit-module-boundary-types': 'off', // Allow inference
    '@typescript-eslint/no-explicit-any': 'off', // Sometimes necessary
    'react/prop-types': 'off', // Docusaurus doesn't always use prop-types
  },
  settings: {
    react: {
      version: 'detect',
    },
  },
  ignorePatterns: ['build', 'dist', '.docusaurus', 'node_modules'],
};

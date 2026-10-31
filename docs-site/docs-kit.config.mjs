// The docs site's configuration: everything docs-kit does not default.
// https://github.com/branes-ai/docs-kit
export default {
  repo: 'scout-drone',
  title: 'Scout Drone',
  description: 'The Branes.AI mission repo for flight to a goal through terrain never seen, without maps.',
  // The session logs are working records, not documentation.
  unpublished: ['sessions'],
  accent: {
    dark: { low: '#14301f', accent: '#3fbf73', high: '#c4f0d4' },
    light: { low: '#d3f2df', accent: '#1d7a45', high: '#0c311c' },
  },
  api: {
    name: 'Scout Drone C++ API',
    brief: 'branes::scout — the scout-drone mission on anatomy',
    input: ['sdk/include'],
  },
};

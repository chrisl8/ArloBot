const { defineConfig } = require("cypress");

module.exports = defineConfig({
  projectId: '9wxj3u',
  e2e: {
    supportFile: false,
    // We've imported your old cypress plugins here.
    // You may want to clean this up later by importing these.
    setupNodeEvents(on, config) {
      return require("./cypress/plugins/index.js")(on, config);
    },
    baseUrl: "http://localhost:5173/",
    testIsolation: false,
  },
});

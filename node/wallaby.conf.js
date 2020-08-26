module.exports = function () {
  return {
    files: [
      '**/*.js',
      { pattern: 'spec/*Spec.js', ignore: true },
      { pattern: 'node_modules/**', ignore: true },
    ],

    tests: ['spec/*Spec.js'],
    env: {
      type: 'node',
      runner: 'node',
    },
  };
};

// Also see .prettierrc
// My goal here is to make this as close to AIRBNB as possible,
// while still using Prettier, which takes care of all the questions and
// keeping things VERY consistent.
// To test things, comment the long extends line, uncomment the one that just has 'airbnb'
// and comment out the prettier rule,
// This will leave you with just AirBNB, and see if it agrees mostly.

// One compromise I made was to ALWAYS use parens on arrow function arguments,
// because it makes both AirBNB and Prettier happy and is automatic.
// Less thinking is better than perfect and/or my preferences.

module.exports = {
  plugins: ['prettier'],
  extends: ['airbnb', 'prettier'],
  env: {
    es6: true, // Probably not needed, because of babel-eslint
    node: true, // Probably not needed, because of babel-eslint
    jest: true,
  },
  rules: {
    'prettier/prettier': 'error',
    // These are not code style or real errors, just "best practices" that really mean
    // me making wonky code to fit requirements I don't need to fulfill.
    // Someday I'll remove these as I get better. ;)
    'class-methods-use-this': 0,
    'no-console': 'off', // As a node.js app, we clearly need console.
    'no-prototype-builtins': 'off', // This seems like overkill
    'prefer-destructuring': 'off',
    'no-plusplus': 'off',
    'no-restricted-syntax': 'off',
  },
};

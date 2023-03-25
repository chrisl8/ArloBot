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

// NOTE: This .eslintrc.cjs file is optimized for a Node.js app. I use slightly different
// rules for Web front end code.

module.exports = {
  parserOptions: {
    ecmaVersion: 2022,
  },
  extends: ['airbnb', 'prettier'],
  rules: {
    'class-methods-use-this': 0,
    'no-console': 'off', // Sometimes we want to, okay?
    'no-prototype-builtins': 'off', // This seems like overkill
    'prefer-destructuring': 'off',
    'no-plusplus': 'off',
    'no-restricted-syntax': 'off',
  },
};

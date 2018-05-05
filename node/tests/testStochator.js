const Stochator = require('stochator');

const die = new Stochator(
  {
    kind: 'integer',
    min: 1,
    max: 6,
  },
  'roll',
);
console.log(die.roll()); // 6
console.log(die.roll()); // 1
console.log(die.roll()); // 2

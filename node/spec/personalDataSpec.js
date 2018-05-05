const fs = require('fs');

const personalDataFile = `${
  process.env.HOME
}/.arlobot/personalDataForBehavior.json`;
const personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

describe('Personal data file tests', () => {
  it('should have been edited with user settings', () => {
    expect(personalData.hasBeenEdited).toBe(true);
  });
});

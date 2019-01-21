const fs = require('fs');

const personalDataFile = `${
  process.env.HOME
}/.arlobot/personalDataForBehavior.json`;
const personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

describe('Personal data file tests', () => {
  it('should not have been edited with user settings yet', () => {
    expect(personalData.hasBeenEdited).toBe(false);
  });
});

// TODO: Update the file and then check again?

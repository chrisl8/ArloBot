var fs = require('fs');
var personalDataFile = process.env.HOME + '/.arlobot/personalDataForBehavior.json';
var personalData = JSON.parse(fs.readFileSync(personalDataFile, 'utf8'));

describe('Personal data file tests', function() {

    it('should have been edited with user settings', function() {
        expect(personalData.hasBeenEdited).toBe(true);
    });
});

const howManySecondsSince = require('../howManySecondsSince');
const wait = require('../wait');

describe('howManySecondsSince', () => {
  it('should return 1 year if given no input', async () => {
    expect(howManySecondsSince()).toBe(60 * 60 * 24 * 365);
  });
  it('should show above 1.0 after waiting 1 second', async () => {
    const oldDate = new Date();
    await wait(1);
    expect(howManySecondsSince(oldDate)).toBeGreaterThanOrEqual(1);
  });
});

const wait = require('../wait');

describe('wait', () => {
  it('should wait for 1000ms', async () => {
    const now = new Date();
    await wait(1);
    const later = new Date();
    expect(later - now).toBeGreaterThanOrEqual(1000);
  });
});

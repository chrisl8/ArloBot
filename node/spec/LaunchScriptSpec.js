const LaunchScript = require('../LaunchScript');

describe('LaunchScript', () => {
  let script;

  beforeEach((done) => {
    script = new LaunchScript({
      // debugging: true,
      name: 'Test',
      scriptName: './spec/testScript.sh',
      callback: done,
    });
    script.start();
  });

  it('should be able to run the script ', () => {
    expect(script.started).toBe(true);
  });

  it('should be able to complete the script with success ', () => {
    expect(script.startupComplete).toBe(true);
  });
});

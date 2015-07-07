describe("LaunchScript", function() {
    var LaunchScript = require('../LaunchScript');
    var script;

    beforeEach(function(done) {
        script = new LaunchScript({
            //debugging: true,
            name: 'Test',
            scriptName: './spec/testScript.sh',
            callback: done
        });
        script.start();
    });

    it("should be able to run the script ", function() {
        expect(script.started).toBe(true);
    });

    it("should be able to complete the script with success ", function() {
        expect(script.startupComplete).toBe(true);
    });
});

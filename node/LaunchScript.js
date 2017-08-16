var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var spawn = require('child_process').spawn;

function LaunchScript(options) {
    this.name = options.name || undefined;
    this.ROScommand = options.ROScommand || undefined;
    this.scriptName = options.scriptName || undefined;
    this.scriptArguments = options.scriptArguments || undefined;
    this.successString = options.successString || undefined;
    this.callback = options.callback || undefined;
    this.started = false;
    this.startupComplete = false;
    this.hasExited = false;
}

LaunchScript.prototype.start = function() {
    this.started = true;
    this.hasExited = false;
    var self = this;
    if (webModel.debugging) {
        console.log('Running ' + this.name + ' child process . . .');
    }
    this.startupComplete = false;
    if (webModel.debugging) {
        console.log(this.name + ' is starting up . . .');
    }
    webModelFunctions.scrollingStatusUpdate(this.name + ' is starting up . . .');
    if (this.ROScommand) {
        this.process = spawn('./runROScommand.sh', [this.ROScommand]);
    } else if (this.scriptName) {
        this.process = spawn(this.scriptName, [this.scriptArguments]);
    }
    this.process.stdout.on('data', (data) => {
        if (webModel.debugging) {
            process.stdout.write(self.name + ' stdout data:' + data);
        }
        if (self.successString && !self.startupComplete) {
            // Report lines before success string found
            // If they are not too long
            if (data.length < 50) {
                // Append to 2nd status line
                webModelFunctions.scrollingStatusUpdate(data);
            }
        }
        if (self.successString) {
            if (data.indexOf(self.successString) > -1) {
                self.startupComplete = true;
                webModelFunctions.scrollingStatusUpdate(self.name + ' successfully started.');
            }
        } else {
            // Assume "success" on any return data!
            self.startupComplete = true;
            webModelFunctions.scrollingStatusUpdate(self.name + ' successfully started.');
        }
    });
    this.process.stderr.on('data', (data) => {
        if (webModel.debugging) {
            console.log(self.name + ' stderr data:' + data);
        }
    });
    this.process.on('error', (err) => {
        if (webModel.debugging) {
            console.log(self.name + ' error:' + err);
        }
        webModelFunctions.scrollingStatusUpdate(self.name + ' process error: ' + err);
    });
    this.process.on('close', (code) => {
        webModelFunctions.scrollingStatusUpdate(self.name + ' exited with code: ' + code);
        self.exitCode = code;
        self.hasExited = true;
        self.startupComplete = true;
        if (webModel.debugging) {
            console.log(self.name + ' exited with code: ' + code);
        }
        if (self.callback) {
            self.callback();
        }
    });
};

module.exports = LaunchScript;

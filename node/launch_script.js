var spawn = require('child_process').spawn;

function LaunchScript(options) {
    this.name = options.name || undefined;
    this.ROScommand = options.ROScommand || undefined;
    this.scriptName = options.scriptName || undefined;
    this.scriptArguments = options.scriptArguments || undefined;
    this.successString = options.successString || undefined;
    this.callback = options.callback || undefined;
    this.debugging = options.debugging || false;
    this.started = false;
    this.startupComplete = false;
    this.hasExited = false;
}

LaunchScript.prototype.start = function() {
    var webserver = require('./webserver'); // Beware of the circular require!
    // NOTE: IF you require this at the TOP of this module, it breaks,
    // basically webserver gets a blank copy of this module
    this.started = true;
    this.hasExited = false;
    var self = this;
    if (this.debugging) {
        console.log('Running ' + this.name + ' child process . . .');
    }
    this.startupComplete = false;
    if (this.debugging) {
        console.log(this.name + ' is starting up . . .');
    }
    webserver.scrollingStatusUpdate(this.name + ' is starting up . . .');
    if (this.ROScommand) {
        this.process = spawn('./runROScommand.sh', [this.ROScommand]);
    } else if (this.scriptName) {
        this.process = spawn(this.scriptName, [this.scriptArguments]);
    }
    this.process.stdout.setEncoding('utf8');
    this.process.stdout.on('data', function(data) {
        if (self.successString && !self.startupComplete) {
            // Report lines before success string found
            // If they are not too long
            if (data.length < 50) {
                // Append to 2nd status line
                webserver.scrollingStatusUpdate(data);
            }
        }
        if (self.successString) {
            if (data.indexOf(self.successString) > -1) {
                self.startupComplete = true;
                webserver.scrollingStatusUpdate(self.name + ' successfully started.');
            }
        } else {
            // Assume "success" on any return data!
            self.startupComplete = true;
            webserver.scrollingStatusUpdate(self.name + ' successfully started.');
        }
        if (self.debugging) {
            process.stdout.write(self.name + ' stdout data:' + data);
        }
    });
    this.process.stderr.setEncoding('utf8');
    this.process.stderr.on('data', function(data) {
        if (self.debugging) {
            console.log(self.name + ' stderr data:' + data);
        }
    });
    this.process.on('error', function(err) {
        if (self.debugging) {
            console.log(self.name + ' error:' + err);
        }
        webserver.scrollingStatusUpdate(self.name + ' process error: ' + err);
    });
    this.process.on('exit', function(code) {
        webserver.scrollingStatusUpdate(self.name + ' exited with code: ' + code);
        self.hasExited = true;
        self.startupComplete = true;
        if (self.debugging) {
            console.log(self.name + ' exited with code: ' + code);
        }
        if (self.callback) {
            self.callback();
        }
    });
};

module.exports = LaunchScript;

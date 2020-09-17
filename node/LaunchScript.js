const spawn = require('child_process').spawn;
const webModel = require('./webModel');
const webModelFunctions = require('./webModelFunctions');

class LaunchScript {
  constructor(options) {
    this.name = options.name;
    this.ROScommand = options.ROScommand;
    this.scriptName = options.scriptName;
    this.scriptArguments = options.scriptArguments;
    this.successString = options.successString;
    this.callback = options.callback;
    this.finalSuccessResultString = options.finalSuccessResultString;
    this.finalFailureResultString = options.finalFailureResultString;
    this.started = false;
    this.startupComplete = false;
    this.hasExited = false;
  }

  start() {
    this.started = true;
    this.hasExited = false;
    if (webModel.debugging || webModel.logConsoleMessages) {
      /* istanbul ignore next */
      console.log(`Running ${this.name} child process . . .`);
    }
    this.startupComplete = false;
    if (webModel.debugging || webModel.logConsoleMessages) {
      /* istanbul ignore next */
      console.log(`${this.name} is starting up . . .`);
    }
    webModelFunctions.scrollingStatusUpdate(
      `${this.name} is starting up . . .`,
    );
    if (this.ROScommand) {
      this.process = spawn('./runROScommand.sh', [this.ROScommand]);
    } else if (this.scriptName) {
      this.process = spawn(this.scriptName, [this.scriptArguments]);
    }
    this.process.stdout.on('data', (data) => {
      if (webModel.debugging || webModel.logConsoleMessages) {
        /* istanbul ignore next */
        console.log(`${this.name}:${data.toString().trim()}`);
      }
      if (!this.startupComplete) {
        if (this.successString) {
          // Report lines before success string found
          // If they are not too long
          if (data.length < 50) {
            // Append to 2nd status line
            webModelFunctions.scrollingStatusUpdate(data);
          }
        }
        if (this.successString) {
          if (data.indexOf(this.successString) > -1) {
            this.startupComplete = true;
            webModelFunctions.scrollingStatusUpdate(
              `${this.name} successfully started.`,
            );
          }
        } else {
          // Assume "success" on any return data!
          this.startupComplete = true;
          webModelFunctions.scrollingStatusUpdate(
            `${this.name} successfully started.`,
          );
        }
      }
      if (
        this.finalSuccessResultString &&
        data.indexOf(this.finalSuccessResultString) > -1
      ) {
        this.finalSuccess = true;
      }
      if (
        this.finalFailureResultString &&
        data.indexOf(this.finalFailureResultString) > -1
      ) {
        this.finalFailure = true;
      }
      if (data.indexOf('GOAL Reached!') > -1) {
        webModelFunctions.update('lastNavigationResult', 'Success');
      }
    });
    this.process.stderr.on('data', (data) => {
      if (webModel.debugging || webModel.logConsoleMessages) {
        /* istanbul ignore next */
        console.log(`${this.name} stderr data:${data}`);
      }
    });
    this.process.on('error', (err) => {
      if (webModel.debugging || webModel.logConsoleMessages) {
        /* istanbul ignore next */
        console.log(`${this.name} error:${err}`);
      }
      webModelFunctions.scrollingStatusUpdate(
        `${this.name} process error: ${err}`,
      );
    });
    this.process.on('close', (code) => {
      webModelFunctions.scrollingStatusUpdate(
        `${this.name} exited with code: ${code}`,
      );
      this.exitCode = code;
      this.hasExited = true;
      this.startupComplete = true;
      if (webModel.debugging || webModel.logConsoleMessages) {
        /* istanbul ignore next */
        console.log(`${this.name} exited with code: ${code}`);
      }
      if (this.callback) {
        this.callback();
      }
    });
  }
}

module.exports = LaunchScript;

const spawn = require('child_process').spawn;
const personalData = require('./personalData');
const webModel = require('./webModel');
const robotModel = require('./robotModel');
const webModelFunctions = require('./webModelFunctions');
const ipAddress = require('./ipAddress');
const masterRelay = require('./MasterRelay');

class Camera {
  /** @namespace personalData.relays.has_fiveVolt */
  /** @namespace personalData.useMasterPowerRelay */
  constructor(cameraName, cameraModel) {
    this.cameraName = cameraName;
    this.cameraModel = cameraModel;
    this.video = '/dev/video0'; // Default, but findAndSwitchOn() will set this.
    this.robotIP = ipAddress.ipAddress();
    this.originalVideoSource = webModel.videoSource;
    this.resolution = personalData.camera0resolutionForWeb || '640x480';
    this.frameRate = personalData.camera0fpsForWeb || 30;
  }

  toggle() {
    if (!personalData.demoWebSite) {
      if (!webModel.cameraOn) {
        this.findAndSwitchOn();
      } else {
        Camera.switchOff();
      }
    } else {
      webModelFunctions.update('cameraOn', !webModel.cameraOn);
    }
  }

  switchOn() {
    webModelFunctions.update('cameraOn', true);
    webModelFunctions.scrollingStatusUpdate(`Starting ${this.cameraName}`);
    // See scripts/streamVideoTest.sh for details on mjpg_streamer usage.
    const process = spawn('/usr/local/bin/mjpg_streamer', [
      '-i',
      `/usr/local/lib/mjpg-streamer/input_uvc.so -d ${this.video} -n -f ${
        this.frameRate
      } -r ${this.resolution}`,
      '-o',
      '/usr/local/lib/mjpg-streamer/output_http.so -p 58180 -w /usr/local/share/mjpg-streamer/www',
    ]);
    process.stdout.on('data', (data) => {
      console.log(`${this.cameraName} stdout: ${data}`);
    });

    process.stderr.on('data', (data) => {
      console.log(`${this.cameraName} stderr: ${data}`);
    });

    process.on('close', (code) => {
      // console.log(`child process exited with code ${code}`);
      if (code === null) {
        webModelFunctions.scrollingStatusUpdate(
          `${this.cameraName} ended normally.`,
        );
      } else {
        console.log(`${this.cameraName} failed with code: ${code}`);
      }
      webModelFunctions.update('cameraOn', false);
      webModelFunctions.update('videoSource', this.originalVideoSource);
    });
    webModelFunctions.update(
      'videoSource',
      `http://${this.robotIP}:58180/?action=stream`,
    );
  }

  static switchOff() {
    spawn('pkill', ['-f', 'mjpg_streamer']);
  }

  findAndSwitchOn() {
    this.dataHolder = '';
    let delayForUsb = 0;
    if (personalData.useMasterPowerRelay && !webModel.masterRelayOn) {
      masterRelay('on');
      delayForUsb = 5;
    }
    console.log(webModel.relays);
    if (
      personalData.relays.has_fiveVolt &&
      !webModel.relays.find((x) => x.name === 'fiveVolt').relayOn
    ) {
      robotModel.usbRelay.switchRelay(
        webModel.relays.find((x) => x.name === 'fiveVolt').number,
        'on',
      );
      delayForUsb = 5;
    }
    if (delayForUsb > 0) {
      webModelFunctions.scrollingStatusUpdate('Camera will be up soon . . .');
    }
    setTimeout(() => {
      webModelFunctions.scrollingStatusUpdate(
        `Finding Camera ${this.cameraModel}`,
      );
      const process = spawn(`${__dirname}/../scripts/find_camera.sh`, [
        this.cameraModel,
      ]);
      process.stdout.on('data', (data) => {
        this.dataHolder = data;
      });
      process.on('close', (code) => {
        // console.log(`child process exited with code ${code}`);
        if (code === 0) {
          this.video = `${this.dataHolder}`.trim();
          this.switchOn();
        } else {
          webModelFunctions.scrollingStatusUpdate(
            `${this.cameraModel} search failed with code: ${code}`,
          );
        }
      });
    }, delayForUsb * 1000);
  }
}

module.exports = Camera;

if (require.main === module) {
  // Run the function if this is called directly instead of required.
  const camera = new Camera('Camera', personalData.camera0name);
  camera.findAndSwitchOn();
}

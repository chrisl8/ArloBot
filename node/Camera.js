var personalData = require('./personalData');
var webModel = require('./webModel');
var webModelFunctions = require('./webModelFunctions');
var robotModel = require('./robotModel');
const spawn = require('child_process').spawn;
var ipAddress = require('./ipAddress');
const masterRelay = require('./MasterRelay');
const UsbRelay = require('./UsbRelayControl');
var usbRelay = new UsbRelay();

module.exports = class Camera {
    constructor(cameraName, model) {
        this.cameraName = cameraName;
        this.model = model;
        this.video = '/dev/video0'; // Default, but findAndSwitchOn() will set this.
        this.robotIP = ipAddress.ipAddress();
        this.originalVideoSource = webModel.videoSource;
    }

    toggle() {
        if (!webModel.cameraOn) {
            this.findAndSwitchOn();
        } else {
            Camera.switchOff();
        }
    }

    switchOn() {
        webModelFunctions.update('cameraOn', true);
        webModelFunctions.scrollingStatusUpdate('Starting ' + this.cameraName);
        /* Logitech c615 resolution is 1280 x 720 (or HD if you want but I don't want to eat the bandwidth)
           http://www.logitech.com/en-us/product/hd-webcam-c615
        */
        const process = spawn('mjpg_streamer', ['-i', '/usr/local/lib/input_uvc.so -d ' + this.video + ' -n -f 30 -r 1280x720', '-o', '/usr/local/lib/output_http.so -p 58180 -w ../scripts/mjpg-streamer/mjpg-streamer/www']);
        process.stdout.on('data', (data) => {
            console.log(`${this.cameraName} stdout: ${data}`);
        });

        process.stderr.on('data', (data) => {
            console.log(`${this.cameraName} stderr: ${data}`);
        });

        process.on('close', (code) => {
            // console.log(`child process exited with code ${code}`);
            if (code === null) {
                webModelFunctions.scrollingStatusUpdate(this.cameraName + ' ended normally.');
            } else {
                console.log(`${this.cameraName} failed with code: ${code}`);
            }
            webModelFunctions.update('cameraOn', false);
            webModelFunctions.update('videoSource', this.originalVideoSource);
        });
        webModelFunctions.update('videoSource', 'http://' + this.robotIP + ':58180/?action=stream');
    }

    static switchOff() {
        spawn('pkill', ['-f', 'mjpg_streamer']);
    }

    findAndSwitchOn() {
        this.dataHolder = '';
        var delayForUsb = 0;
        if (personalData.useMasterPowerRelay && !webModel.masterRelayOn) {
            masterRelay('on');
            delayForUsb = 5;
        }
        if (personalData.relays.has_fiveVolt && !webModel.relays.find(x=> x.name === 'fiveVolt')['relayOn']) {
            usbRelay.switch(webModel.relays.find(x=> x.name === 'fiveVolt')['number'], 'on');
            delayForUsb = 5;
        }
        if (delayForUsb > 0) {
            webModelFunctions.scrollingStatusUpdate('Camera will be up soon . . .');
        }
        setTimeout(() => {
            webModelFunctions.scrollingStatusUpdate('Finding Camera ' + this.model);
            const process = spawn(__dirname + '/../scripts/find_camera.sh', [this.model]);
            process.stdout.on('data', (data) => {
                this.dataHolder = data;
            });
            process.on('close', (code) => {
                // console.log(`child process exited with code ${code}`);
                if (code === 0) {
                    this.video = this.dataHolder;
                    this.switchOn();
                } else {
                    webModelFunctions.scrollingStatusUpdate(`${this.model} search failed with code: ${code}`);
                }
            });
        }, delayForUsb * 1000);
    }
};
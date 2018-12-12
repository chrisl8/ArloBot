/* eslint-disable no-param-reassign */
const spawn = require('child_process').spawn;
const fs = require('fs');

class UsbDevice {
  constructor(uniqueDeviceString, stringLocation) {
    this.uniqueDeviceString = uniqueDeviceString;
    // stringLocation tells what line of the udevadm output the uniqueDeviceString is found in.
    // Usually 'product', 'name' or 'manufacturer'
    this.stringLocation = stringLocation;
  }

  findDeviceName() {
    return new Promise((resolve, reject) => {
      this.getLinuxUsbDeviceList()
        .then((deviceList) => this.getInfoFromDeviceList(deviceList))
        .then((infoDump) => {
          let foundDevice = false;
          let deviceName;
          for (let i = 0; i < infoDump.length; i++) {
            for (let j = 0; j < infoDump[i].deviceInfo.length; j++) {
              if (infoDump[i].deviceInfo[j].includes(this.stringLocation)) {
                const deviceStringLine = infoDump[i].deviceInfo[j].split('=');
                if (deviceStringLine.length > 0) {
                  const re = /"/g;
                  infoDump[i].deviceString = deviceStringLine[1].replace(
                    re,
                    '',
                  );
                }
                break;
              }
            }
            if (infoDump[i].hasOwnProperty('deviceString')) {
              if (infoDump[i].deviceString.includes(this.uniqueDeviceString)) {
                foundDevice = true;
                deviceName = `/dev/${infoDump[i].device}`;
                break;
              }
            }
          }
          if (foundDevice) {
            resolve(deviceName);
          } else {
            reject(new Error('Not found.'));
          }
        })
        .catch((error) => {
          console.log(`ERROR: ${error}`);
        });
    });
  }

  getLinuxUsbDeviceList() {
    return new Promise((resolve, reject) => {
      // all /dev/ttyUSB* and /dev/ttyACM* file names
      fs.readdir('/dev/', (err, list) => {
        if (err) {
          reject(err);
        } else {
          const outputList = [];
          const interestingDeviceNames = ['USB', 'ACM'];
          for (let i = 0; i < list.length; i++) {
            for (let j = 0; j < interestingDeviceNames.length; j++) {
              if (list[i].includes(interestingDeviceNames[j])) {
                outputList.push(list[i]);
              }
            }
          }
          resolve(outputList);
        }
      });
    });
  }

  getInfoFromDeviceList(deviceList) {
    // eslint-disable-next-line arrow-body-style
    const getSingleDeviceInfo = (device) => {
      return new Promise((resolve, reject) => {
        let outputData = '';
        const process = spawn('udevadm', ['info', '-n', `/dev/${device}`]);
        process.stdout.on('data', (data) => {
          outputData += data;
        });
        process.stderr.on('data', (data) => {
          console.log(`stderr: ${data}`);
        });
        process.on('close', (code) => {
          if (code === null || code === 0) {
            const outputAsArray = String(outputData).split('\n');
            resolve({
              device,
              deviceInfo: outputAsArray,
            });
          } else {
            reject(code);
          }
        });
      });
    };
    const allResults = deviceList.map((device) =>
      getSingleDeviceInfo(device).then((deviceInfo) => deviceInfo),
    );
    return Promise.all(allResults);
  }
}

module.exports = UsbDevice;
if (require.main === module) {
  // Run the function if this is called directly instead of required.
  if (process.argv.length < 4) {
    console.log('You must provide a string to search for,');
    console.log(
      "and the line it is contained in, usually 'product', 'name' or 'manufacturer'.",
    );
    console.log('i.e.');
    console.log(
      'node UsbDevice.js "Numato Lab 1 Channel USB Powered Relay Module" product',
    );
    process.exit();
  }
  const usbDevice = new UsbDevice(process.argv[2], process.argv[3]);
  usbDevice
    .findDeviceName()
    .then((deviceName) => {
      console.log(`${deviceName}`);
    })
    .catch((error) => {
      console.log(`ERROR: ${error}`);
      process.exit(1);
    });
}

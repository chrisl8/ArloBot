// Numato Lab - http://numato.com
// https://github.com/numato/samplecode/blob/master/RelayAndGPIOModules/USBRelayAndGPIOModules/node.js/usbrelay/UsbRelay.js
// This node.js sample script opens the port and sends two commands to the device. These commands
// will turn on Relay0, wait for 2 seconds and then turn off. The response received from the device
// is printed to the console unparsed.
// Please follow the steps below to test the script.
//
// 1. Download and install Node.js from https://nodejs.org/download/
// 2. Run command "npm install serialport" at the command prompt to install serial port library.
// 3. Attach the gpio device to the PC and note the port identifier corresponding to the device.
// 4. Update the line below that starts with "var port =" with the port name for your device.
// 5. Run the script by entering the command "node UsbRelay" at the command prompt

var SerialPort = require("serialport").SerialPort;
if (process.argv.length < 4) {
    console.log("You must provide the port and on or off, for example:");
    console.log("node UsbRelay.js $(../scripts/find_MasterRelayBoard.sh) off");
    process.exit();
}
var port = process.argv[2];
var operation = process.argv[3];

var portObj = new SerialPort(port,{
  baudrate: 19200
}, false);

portObj.on('data', function(data){
    console.log('Data Returned by the device');
    console.log('--------------------');
    console.log(String(data));
    console.log('--------------------');
});

portObj.open(function (error){
  if ( error ) {
        console.error('Failed to open port: ' + error);
        process.exit(1);
  } else {
        portObj.write("relay " + operation + " 0\r", function(err, results){
            if(error){
                console.error('Failed to write to port: '+ error);
                process.exit(1);
            }
            // For some reason after a fresh reboot,
            // it only works if I do it twice.
            portObj.write("relay " + operation + " 0\r", function(err, results){
                portObj.close();
            });
        });
  }
});

/* Here is the "source" in the Angular 2 TypeScript for roslib.js
 * This is based on SocketService.service.ts, so view that if you are trying to understand
 * how to do it right. That came from another source online somewhere.
 *
 * Be sure to install the .min.js file
 * (Download or npm?)
 * TODO: Add it to NPM if we can find the actual roslib.min.js file
 * This is also required by roslib:
 *<script type="text/javascript" src="eventemitter2.min.js"></script>
 *<script type="text/javascript" src="roslib.min.js"></script>
 * after all of the other <script> lines.
 * 
 *
 * You CAN put the URL ino the "new" line like:
 *  = new ROSLIB.Ros({
 *  url: 'ws://localhost:9090',
 *  // This eliminates a warning about utf8:
 *  encoding: 'ascii'
 *  });
 * , but since it does not auto
 * retry, it seems best to put it in the connect line in the calling
 * code.
 */

// Fake out TypeScript regarding a generic JS library by declaring a bogus variable:
declare var ROSLIB:any;

let socket = new ROSLIB.Ros();
let Message = ROSLIB.Message;

export class RosService {
    socket;
    Message;

    constructor() {
        this.socket = socket;
        this.Message = Message;
    }

    emit(type:string) {
        this.socket.emit(type);
    }

    emitData(type:string, data:string) {
        this.socket.emit(type, data);
    }

    twist(linear_speed:number, angular_speed:number) {
        return new this.Message({
            linear: {
                x: linear_speed,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: angular_speed
            }
        });
    }

    // Setup ROS cmdVel Publishing a Topic for the TeleOp function
    cmdVel() {
        return new ROSLIB.Topic({ // "defined" in the declare statement above the class.
            ros: socket, // Uses the same socket we already declared.
            name: '/cmd_vel_mux/input/web',
            messageType: 'geometry_msgs/Twist'
        });
    }

}
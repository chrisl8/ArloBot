import ROSLIB from 'roslib';

class RosService {
  constructor() {
    this.socket = new ROSLIB.Ros();
    this.Message = ROSLIB.Message;
    // For testing running locally while connecting site to a remote robot:
    // this.hostname = `ws://twoflower.local:8080`;
    // For production:
    this.hostname = `ws://${window.location.hostname}:9090`;
  }

  emit(type) {
    this.socket.emit(type);
  }

  emitData(type, data) {
    this.socket.emit(type, data);
  }

  twist(linearSpeed, angularSpeed) {
    return new this.Message({
      linear: {
        x: linearSpeed,
        y: 0.0,
        z: 0.0,
      },
      angular: {
        x: 0.0,
        y: 0.0,
        z: angularSpeed,
      },
    });
  }

  // Setup ROS cmdVel Publishing a Topic for the TeleOp function
  cmdVel() {
    return new ROSLIB.Topic({
      ros: this.socket,
      name: '/cmd_vel_mux/input/web',
      messageType: 'geometry_msgs/Twist',
    });
  }
}

export default RosService;

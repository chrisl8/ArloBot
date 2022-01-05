import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import nipplejs from 'nipplejs';
import './RemoteControl.css';
// This is the only component that uses the ROS Service, so I'm importing it and setting it up here
import RosService from '../utils/RosService';
import boolToYesNo from '../utils/boolToYesNo';

class RemoteControl extends Component {
  constructor(props) {
    super(props);
    this.toggle = this.toggle.bind(this);
    this.state = {
      isOpen: false,
      joystickOutput: 'Use finger or mouse to drive robot!',
    };
    this.rosConnected = false;
  }

  componentDidMount() {
    this.RosService = new RosService();
    this.RosService.socket.on('connection', () => {
      this.cmdVel = this.RosService.cmdVel();
      this.rosConnected = true;
      // console.log('RosLibJS Connected');
      this.props.sendDataToRobot('scrollingStatusUpdate', 'RosLibJS Connected');
    });
    this.RosService.socket.on('error', (error) => {
      this.rosConnected = false;
      this.cmdVel = undefined;
      // console.log('Error connecting to RosLibJS server: ', error);
      this.props.sendDataToRobot(
        'scrollingStatusUpdate',
        `Error connecting to RosLibJS server: ${error}`,
      );
    });
    this.RosService.socket.on('close', () => {
      this.rosConnected = false;
      this.cmdVel = undefined;
      // this.cmdVel = undefined; // I'm not sure if this is needed, but it seems smart.
      // console.log('RosLibJS Disconnected.');
      this.props.sendDataToRobot(
        'scrollingStatusUpdate',
        'RosLibJS Disconnected.',
      );
    });
  }

  componentDidUpdate() {
    if (this.state.isOpen) {
      if (!this.rosConnected && this.props.ROSisRunning) {
        this.rosConnected = true;
        try {
          this.RosService.socket.connect(this.RosService.hostname);
        } catch (e) {
          // eslint-disable-next-line no-console
          console.log(e);
        }
      }
      if (!this.joystick) {
        const options = {
          // zone: document.getElementById('virtual-joystick-container')
          zone: this.joystickContainer,
        };

        // Create new joystick
        this.joystick = nipplejs.create(options);

        /* Use this to dump ALL data it can give you for testing.
        this.joystick.on('start end', function (evt, data) {
            console.log(evt.type);
            console.log(data);
        }).on('move', function (evt, data) {
            console.log(evt.type);
            console.log(data);
        }).on('dir:up plain:up dir:left plain:left dir:down ' +
            'plain:down dir:right plain:right',
            function (evt, data) {
                console.log(evt.type);
                console.log(data);
            }
        ).on('pressure', function (evt, data) {
            console.log(evt.type);
            console.log({
                pressure: data
            });
        });
        */

        // Create event listeners and event handlers for new joystick
        this.joystick
          .on('start end', () => {
            // evt, data
            // console.log(evt.type);
            // console.log(data);
            const linearSpeed = 0;
            const angularSpeed = 0;
            this.setState({
              joystickOutput: `LinearSpeed: ${linearSpeed}, AngularSpeed: ${angularSpeed}`,
            });
            this.sendTwistCommandToROS(linearSpeed, angularSpeed);
          })
          .on('move', (evt, data) => {
            // console.log(evt.type);
            // console.log(data);
            // console.log(data.angle.radian);
            // console.log(data.distance);
            // https://math.stackexchange.com/questions/143932/calculate-point-given-x-y-angle-and-distance
            // x=5cosθ , y=5sinθ.
            // θ is the angle between the line of sight from the entity
            // to the point and the positive x axis.
            // console.log(Math.sin(45 * (Math.PI / 180)) * 5);
            const joystickXfromCenter =
              Math.cos(data.angle.radian) * data.distance;
            const joystickYfromCenter =
              Math.sin(data.angle.radian) * data.distance;
            const DECREASER = 100;
            const linearSpeed =
              Math.round((joystickYfromCenter / DECREASER) * 100) / 100;
            // Angular is reversed.
            const angularSpeed =
              Math.round((-joystickXfromCenter / DECREASER) * 100) / 100;
            this.setState({
              joystickOutput: `LinearSpeed: ${linearSpeed}, AngularSpeed: ${angularSpeed}`,
            });
            this.sendTwistCommandToROS(linearSpeed, angularSpeed);
          });
      }
    } else if (this.joystick) {
      this.joystick.destroy();
      this.joystick = null;
    }
    // console.log(this.joystick);
    // console.log(this.joystickContainer.id);
  }

  componentWillUnmount() {
    this.clearRepeater();
  }

  sendTwistCommandToROS(linearSpeed, angularSpeed) {
    // We'll use it to stop too!
    // The best way to see if this is working is:
    // rostopic echo /cmd_vel_mux/input/web # From the robot itself.

    this.clearRepeater();

    if (this.rosConnected && this.cmdVel !== undefined) {
      const twist = this.RosService.twist(linearSpeed, angularSpeed);
      this.cmdVel.publish(twist); // cmdVel is assigned in startROSfunctions()
      if (linearSpeed !== 0 || angularSpeed !== 0) {
        this.repeater = setTimeout(() => {
          this.sendTwistCommandToROS(linearSpeed, angularSpeed);
        }, 500);
      }
    }
  }

  clearRepeater() {
    if (this.repeater) {
      clearTimeout(this.repeater);
    }
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  render() {
    return (
      <Card id="remote-control-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Remote Control</CardTitle>
        </CardHeader>
        <Collapse id="remote-control-card-body" isOpen={this.state.isOpen}>
          <CardBody>
            {!this.props.ROSisRunning && (
              <button
                type="button"
                className="btn btn-danger"
                onClick={() => this.props.openGroup('startupShutdown')}
              >
                ROS Stopped
              </button>
            )}
            {Boolean(
              (!this.props.monitorACconnection && this.props.pluggedIn) ||
                this.props.pluggedIn === true,
            ) && (
              <button
                type="button"
                className="btn btn-danger"
                onClick={() => this.props.openGroup('startupShutdown')}
              >
                Plugged In&nbsp;
                <span className="badge rounded-pill bg-dark">
                  {boolToYesNo(this.props.pluggedIn)}
                </span>
              </button>
            )}
            {this.state.joystickOutput}
            <span id="virtual-joystick-result" />
            <div
              id="virtual-joystick-container"
              ref={(div) => {
                this.joystickContainer = div;
              }}
            />
            If robot does not respond, check the Telemetry section. You can
            override sensors in the Sensors section if you need to.
          </CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default RemoteControl;

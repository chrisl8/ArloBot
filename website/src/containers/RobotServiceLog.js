import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToOnOff from '../utils/boolToOnOff';

class RobotServiceLog extends Component {
  constructor(props) {
    super(props);
    this.toggle = this.toggle.bind(this);
    this.state = { isOpen: false };
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  goToLogStreamer() {
    window.open(`http://${window.location.hostname}:28778/`, '_blank');
  }

  render() {
    let startRosLogStreamerClass = 'btn';
    let startRosLogStreamerBadgeClass = 'badge badge-secondary';
    if (this.props.logStreamerRunning) {
      startRosLogStreamerClass = 'btn btn-success';
      startRosLogStreamerBadgeClass = 'badge badge-light';
    }
    return (
      <Card id="status-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Robot Service Log</CardTitle>
        </CardHeader>
        <Collapse isOpen={this.state.isOpen}>
          <CardBody>
            <div
              id="statusScrollBox"
              dangerouslySetInnerHTML={{ __html: this.props.scrollingStatus }}
            />
            <div style={{ marginTop: '10px' }}>
              {this.props.logStreamerRunning && (
                <button
                  type="button"
                  className="btn btn-primary"
                  onClick={() => this.goToLogStreamer()}
                >
                  View ROS Log Streamer&nbsp;
                </button>
              )}
              {this.props.ROSisRunning && (
                <button
                  type="button"
                  className={startRosLogStreamerClass}
                  onClick={() =>
                    this.props.sendDataToRobot('toggleLogStreamer')
                  }
                >
                  ROS Log Streamer&nbsp;
                  <span className={startRosLogStreamerBadgeClass}>
                    {boolToOnOff(this.props.logStreamerRunning)}
                  </span>
                </button>
              )}
            </div>
          </CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default RobotServiceLog;

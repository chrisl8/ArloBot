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
    window.open(`http://${window.location.hostname}:6688/`, '_blank');
  }

  render() {
    let startRosLogStreamerClass = 'btn';
    let startRosLogStreamerBadgeClass = 'badge badge-secondary';
    if (this.props.logStreamerRunning) {
      startRosLogStreamerClass = 'btn btn-success';
      startRosLogStreamerBadgeClass = 'badge badge-light';
    }
    const scrollingStatus = this.props.scrollingStatus.map((entry, index) => {
      return (
        // eslint-disable-next-line react/no-array-index-key
        <span key={index}>
          {entry}
          <br />
        </span>
      );
    });
    return (
      <Card id="robot-service-log-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Robot Service Log</CardTitle>
        </CardHeader>
        <Collapse id="robot-service-log-card-body" isOpen={this.state.isOpen}>
          <CardBody>
            <div id="statusScrollBox">{scrollingStatus}</div>
            <div style={{ marginTop: '10px' }}>
              {this.props.logStreamerRunning && (
                <button
                  id="view-log-streamer-button"
                  type="button"
                  className="btn btn-primary"
                  onClick={() => this.goToLogStreamer()}
                >
                  View ROS Log Streamer&nbsp;
                </button>
              )}
              {this.props.ROSisRunning && (
                <button
                  id="log-streamer-button"
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

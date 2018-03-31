import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

class StartupShutdown extends Component {
  componentDidMount() {
    this.props.handleUpdateStartupElement(this.cardDiv);
  }

  render() {
    let cardTitleText = <span>Startup/Shutdown</span>;
    if (this.props.ROSisRunning) {
      cardTitleText = (
        <span>
          Startup/Shutdown -{' '}
          <span style={{ fontWeight: 'bold' }}>ROS is Running.</span>
        </span>
      );
    } else if (!this.props.ROSisRunning && !this.props.ROSstart) {
      cardTitleText = (
        <span>
          Startup/Shutdown -{' '}
          <span style={{ fontWeight: 'bold' }}>ROS Stopped.</span>
        </span>
      );
    } else if (!this.props.ROSisRunning && this.props.ROSstart) {
      cardTitleText = (
        <span>
          Startup/Shutdown -{' '}
          <span style={{ fontWeight: 'bold' }}>
            Please wait, ROS is starting . . .
          </span>
        </span>
      );
    }
    let doorsClosedClass = 'btn';
    if (this.props.doorsOpen) {
      doorsClosedClass = 'btn btn-warning';
    }

    // Unplug yourself button
    let unplugButtonClass = 'toggle';
    let unplugButtonLeftSideLabelClass = 'toggler';
    let unplugButtonRightSideLabelClass = 'toggler';
    let unplugButtonValueToSend = 'doNotUnplugYourself';
    if (this.props.unplugYourself) {
      unplugButtonClass += ' toggle-on';
      unplugButtonRightSideLabelClass += ' brightly-negative-text';
    } else {
      unplugButtonValueToSend = 'unplugYourself';
      unplugButtonClass += ' toggle-off';
      unplugButtonLeftSideLabelClass += ' brightly-positive-text';
    }

    return (
      <div
        ref={(element) => {
          this.cardDiv = element;
        }}
      >
        <Card id="status-card" className="card-title">
          <CardHeader onClick={() => this.props.toggle('startupShutdown')}>
            <CardTitle>{cardTitleText}</CardTitle>
          </CardHeader>
          <Collapse isOpen={this.props.isOpen}>
            <CardBody>
              {!this.props.ROSisRunning &&
                !this.props.ROSstart && (
                  <button
                    type="button"
                    className="btn btn-success"
                    onClick={() => this.props.sendDataToRobot('startROS')}
                  >
                    Start ROS
                  </button>
                )}
              {!this.props.ROSisRunning &&
                this.props.ROSstart && (
                  <button type="button" className="btn">
                    Starting...
                  </button>
                )}
              {this.props.ROSisRunning && (
                <span>
                  <button
                    type="button"
                    className="btn btn-danger"
                    onClick={() => this.props.sendDataToRobot('stopROS')}
                  >
                    Stop ROS
                  </button>
                  <button
                    type="button"
                    className={doorsClosedClass}
                    onClick={() =>
                      this.props.sendDataToRobot('markDoorsClosed')
                    }
                  >
                    Set All Doors Closed
                  </button>
                </span>
              )}
              <button
                type="button"
                className="btn btn-danger"
                onClick={() => this.props.sendDataToRobot('exit')}
              >
                Reset Robot Server
              </button>
              <div
                className="lcarish-toggle-button no-flex"
                id="unplug-yourself-button"
                onClick={() =>
                  this.props.sendDataToRobot(unplugButtonValueToSend)
                }
              >
                <label className={unplugButtonLeftSideLabelClass}>No</label>
                <div className={unplugButtonClass}>
                  <input
                    type="checkbox"
                    className="check"
                    checked={this.props.unplugYourself}
                    readOnly
                  />
                  <span className="b switch">Unplug</span>
                </div>
                <label className={unplugButtonRightSideLabelClass}>Yes</label>
              </div>
            </CardBody>
          </Collapse>
        </Card>
      </div>
    );
  }
}

export default StartupShutdown;

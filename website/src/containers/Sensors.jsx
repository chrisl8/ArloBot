import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

class Sensors extends Component {
  constructor(props) {
    super(props);
    this.toggle = this.toggle.bind(this);
    this.state = { isOpen: false };
    this.rosParameterList = [
      {
        index: 2,
        title: 'Infrared',
        status: 'ignoreIRSensors',
        sendIfTrue: 'monitorIR',
        sendIfFalse: 'ignoreIR',
      },
      {
        index: 3,
        title: 'Cliff',
        status: 'ignoreCliffSensors',
        sendIfTrue: 'monitorCliff',
        sendIfFalse: 'ignoreCliff',
      },
      {
        index: 4,
        title: 'Floor',
        status: 'ignoreFloorSensors',
        sendIfTrue: 'monitorFloor',
        sendIfFalse: 'ignoreFloor',
      },
      {
        index: 5,
        title: 'PING',
        status: 'ignoreProximity',
        sendIfTrue: 'monitorProximity',
        sendIfFalse: 'ignoreProximity',
      },
    ];
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  render() {
    const sensorSwitches = this.rosParameterList.map((entry) => {
      let buttonClass = 'toggle toggle-off';
      let buttonLeftSideLabelClass = 'toggler';
      let buttonRightSideLabelClass = 'toggler';
      let buttonValueToSend = entry.sendIfFalse;
      if (this.props[entry.status]) {
        buttonClass = 'toggle toggle-on';
        buttonRightSideLabelClass += ' brightly-negative-text';
        buttonValueToSend = entry.sendIfTrue;
      } else {
        buttonLeftSideLabelClass += ' brightly-positive-text';
      }
      return (
        <div
          key={entry.index}
          className="lcarish-toggle-button no-flex"
          id={entry.status}
          onClick={() => this.props.sendDataToRobot(buttonValueToSend)}
        >
          {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
          <label className={buttonLeftSideLabelClass}>Monitor</label>
          <div className={buttonClass}>
            <input
              type="checkbox"
              className="check"
              checked={this.props[entry.status]}
              readOnly
            />
            <span className="b switch">{entry.title}</span>
          </div>
          {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
          <label className={buttonRightSideLabelClass}>Ignore</label>
        </div>
      );
    });

    return (
      <Card id="sensors-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Sensors</CardTitle>
        </CardHeader>
        <Collapse id="sensors-card-body" isOpen={this.state.isOpen}>
          <CardBody>
            {!this.props.ROSisRunning && (
              <p>These do not work until ROS is Running.</p>
            )}
            {this.props.ROSisRunning && sensorSwitches}
          </CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default Sensors;

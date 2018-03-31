import React, { Component } from 'react';

import EmergencyStopButton from './EmergencyStopButton';

class Banner extends Component {
  render() {
    // Doing if/else in REACT https://www.robinwieruch.de/conditional-rendering-react

    if (this.props.status === 'Robot Offline') {
      return <h1 className="robot-offline">Robot is Offline!</h1>;
    }
    return (
      <p id="status-line">
        {this.props.status}&nbsp;&nbsp;{this.props.behaviorStatus}
        <EmergencyStopButton
          haltRobot={this.props.haltRobot}
          sendDataToRobot={this.props.sendDataToRobot}
        />
      </p>
    );
  }
}

export default Banner;

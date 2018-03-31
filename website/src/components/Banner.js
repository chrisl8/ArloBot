import React from 'react';

import EmergencyStopButton from './EmergencyStopButton';

const Banner = (props) => {
  // Doing if/else in REACT https://www.robinwieruch.de/conditional-rendering-react
  if (props.status === 'Robot Offline') {
    return <h1 className="robot-offline">Robot is Offline!</h1>;
  }
  return (
    <p id="status-line">
      {props.status}&nbsp;&nbsp;{props.behaviorStatus}
      <EmergencyStopButton
        haltRobot={props.haltRobot}
        sendDataToRobot={props.sendDataToRobot}
      />
    </p>
  );
};

export default Banner;

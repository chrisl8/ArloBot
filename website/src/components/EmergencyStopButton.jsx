import React from 'react';
import './EmergencyStopButton.css';

const EmergencyStopButton = (props) => {
  const btnClass = props.haltRobot
    ? 'btn btn-success btn-tight'
    : 'btn btn-danger btn-tight';
  const btnText = props.haltRobot ? 'Resume' : 'Emergency STOP';
  const signal = props.haltRobot ? 'unHaltRobot' : 'haltRobot';

  return (
    <span className="emergency-button">
      <button
        type="button"
        className={btnClass}
        onClick={() => props.sendDataToRobot(signal)}
      >
        {btnText}
      </button>
    </span>
  );
};

export default EmergencyStopButton;

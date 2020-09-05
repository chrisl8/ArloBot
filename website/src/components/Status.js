import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToYesNo from '../utils/boolToYesNo';
import boolToOnOff from '../utils/boolToOnOff';
import boolToUpDown from '../utils/boolToUpDown';
import './Status.css';

const Status = (props) => {
  let laptopBatteryClass = 'btn btn-success';
  if (props.laptopBatteryPercentage < 11) {
    laptopBatteryClass = 'btn btn-danger';
  } else if (props.laptopBatteryPercentage < 50) {
    laptopBatteryClass = 'btn btn-warning';
  }

  let laptopFullyChargedClass = 'btn';
  let laptopFullyChargedBadgeClass = 'badge badge-secondary';
  if (props.laptopFullyCharged === true) {
    laptopFullyChargedClass = 'btn btn-success';
    laptopFullyChargedBadgeClass = 'badge badge-light';
  }

  let pluggedInClass = 'btn';
  let pluggedInBadgeClass = 'badge badge-secondary';
  if (props.pluggedIn) {
    pluggedInClass = 'btn btn-danger';
    pluggedInBadgeClass = 'badge badge-light';
  }

  let mapClass = 'btn';
  let mapBadgeClass = 'badge badge-secondary';
  let mapBadgeText = 'None';
  if (props.mapName !== '') {
    mapClass = 'btn btn-success';
    mapBadgeClass = 'badge badge-light';
    mapBadgeText = props.mapName;
  }

  let debuggingClass = 'btn';
  let debuggingBadgeClass = 'badge badge-secondary';
  if (props.debugging) {
    debuggingClass = 'btn btn-warning';
    debuggingBadgeClass = 'badge badge-light';
  }

  let logConsoleMessagesClass = 'btn';
  let logConsoleMessagesBadgeClass = 'badge badge-secondary';
  if (props.logConsoleMessages) {
    logConsoleMessagesClass = 'btn btn-warning';
    logConsoleMessagesBadgeClass = 'badge badge-light';
  }

  let cameraClass = 'btn';
  let cameraBadgeClass = 'badge badge-secondary';
  if (props.cameraOn) {
    cameraClass = 'btn btn-success';
    cameraBadgeClass = 'badge badge-light';
  }

  let masterRelayClass = 'btn';
  let masterRelayBadgeClass = 'badge badge-secondary';
  if (props.masterRelayOn) {
    masterRelayClass = 'btn btn-success';
    masterRelayBadgeClass = 'badge badge-light';
  }

  let myCroftClass = 'btn';
  let myCroftBadgeClass = 'badge badge-secondary';
  if (props.myCroftIsRunning) {
    myCroftClass = 'btn btn-success';
    myCroftBadgeClass = 'badge badge-light';
  }

  let cloudServerClass = 'btn';
  let cloudServerBadgeClass = 'badge badge-secondary';
  if (props.cloudServerConnected) {
    cloudServerClass = 'btn btn-success';
    cloudServerBadgeClass = 'badge badge-light';
  }

  return (
    <Card id="status-card" className="card-title">
      <CardHeader onClick={() => props.toggle('status')}>
        <CardTitle>Status</CardTitle>
      </CardHeader>
      <Collapse id="status-card-body" isOpen={props.isOpen}>
        <CardBody>
          <button
            id="laptopBatteryStatusButton"
            type="button"
            className={laptopBatteryClass}
          >
            Laptop Battery&nbsp;
            <span className="badge badge-light">
              {props.laptopBatteryPercentage}%
            </span>
          </button>
          <button
            id="laptopChargedStatusButton"
            type="button"
            className={laptopFullyChargedClass}
          >
            Laptop Fully Charged&nbsp;
            <span className={laptopFullyChargedBadgeClass}>
              {boolToYesNo(props.laptopFullyCharged)}
            </span>
          </button>
          <button
            id="pluggedInStatusButton"
            type="button"
            className={pluggedInClass}
            onClick={() => props.openGroup('startupShutdown')}
          >
            Plugged In&nbsp;
            <span className={pluggedInBadgeClass}>
              {boolToYesNo(props.pluggedIn)}
            </span>
          </button>
          <button
            id="mapStatusButton"
            type="button"
            className={mapClass}
            onClick={() => props.openGroup('navigation')}
          >
            Map&nbsp;
            <span className={mapBadgeClass}>{mapBadgeText}</span>
          </button>
          <button
            id="debuggingStatusButton"
            type="button"
            className={debuggingClass}
            onClick={() => props.sendDataToRobot('toggleDebug')}
          >
            Debugging&nbsp;
            <span className={debuggingBadgeClass}>
              {boolToYesNo(props.debugging)}
            </span>
          </button>
          <button
            id="logConsoleMessagesStatusButton"
            type="button"
            className={logConsoleMessagesClass}
            onClick={() => props.sendDataToRobot('toggleLogConsoleMessages')}
          >
            Log Console&nbsp;
            <span className={logConsoleMessagesBadgeClass}>
              {boolToYesNo(props.logConsoleMessages)}
            </span>
          </button>
          <button
            id="cameraStatusButton"
            type="button"
            className={cameraClass}
            onClick={() => props.openGroup('video')}
          >
            Camera&nbsp;
            <span className={cameraBadgeClass}>
              {boolToOnOff(props.cameraOn)}
            </span>
          </button>
          <button
            id="masterRelayStatusButton"
            type="button"
            className={masterRelayClass}
            onClick={() => props.sendDataToRobot('toggleMasterRelay')}
          >
            Master Relay&nbsp;
            <span className={masterRelayBadgeClass}>
              {boolToOnOff(props.masterRelayOn)}
            </span>
          </button>
          <button
            id="mycroftStatusButton"
            type="button"
            className={myCroftClass}
          >
            Mycroft&nbsp;
            <span className={myCroftBadgeClass}>
              {boolToUpDown(props.myCroftIsRunning)}
            </span>
          </button>
          <button
            id="cloudStatusButton"
            type="button"
            className={cloudServerClass}
          >
            Cloud&nbsp;
            <span className={cloudServerBadgeClass}>
              {boolToUpDown(props.cloudServerConnected)}
            </span>
          </button>
        </CardBody>
      </Collapse>
    </Card>
  );
};

export default Status;

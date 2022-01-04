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

  let laptopFullyChargedClass = 'btn btn-light';
  let laptopFullyChargedBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.laptopFullyCharged === true) {
    laptopFullyChargedClass = 'btn btn-success';
    laptopFullyChargedBadgeClass = 'badge rounded-pill bg-dark';
  }

  let pluggedInClass = 'btn btn-light';
  let pluggedInBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.pluggedIn) {
    pluggedInClass = 'btn btn-danger';
    pluggedInBadgeClass = 'badge rounded-pill bg-dark';
  }

  let mapClass = 'btn btn-light';
  let mapBadgeClass = 'badge rounded-pill bg-secondary';
  let mapBadgeText = 'None';
  if (props.mapName !== '') {
    mapClass = 'btn btn-success';
    mapBadgeClass = 'badge rounded-pill bg-dark';
    mapBadgeText = props.mapName;
  }

  let debuggingClass = 'btn btn-light';
  let debuggingBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.debugging) {
    debuggingClass = 'btn btn-warning';
    debuggingBadgeClass = 'badge rounded-pill bg-dark';
  }

  let cameraClass = 'btn btn-light';
  let cameraBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.cameraOn) {
    cameraClass = 'btn btn-success';
    cameraBadgeClass = 'badge rounded-pill bg-dark';
  }

  let masterRelayClass = 'btn btn-light';
  let masterRelayBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.masterRelayOn) {
    masterRelayClass = 'btn btn-success';
    masterRelayBadgeClass = 'badge rounded-pill bg-dark';
  }

  let myCroftClass = 'btn btn-light';
  let myCroftBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.myCroftIsRunning) {
    myCroftClass = 'btn btn-success';
    myCroftBadgeClass = 'badge rounded-pill bg-dark';
  }

  let cloudServerClass = 'btn btn-light';
  let cloudServerBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.cloudServerConnected) {
    cloudServerClass = 'btn btn-success';
    cloudServerBadgeClass = 'badge rounded-pill bg-dark';
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
            <span className="badge rounded-pill bg-dark">
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
          {props.personalData.camera0 && (
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
          )}
          {props.useMasterPowerRelay && (
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
          )}
          {props.useMyCroft && (
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
          )}
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
          {props.debugging && (
            <p>
              Active Debugging Log types:
              <br />
              Console&nbsp;
              <input
                name="console-log"
                type="checkbox"
                checked={props.logConsoleMessages}
                onClick={() =>
                  props.sendDataToRobot('toggleLogConsoleMessages')
                }
              />
              <br />
              Behavior&nbsp;
              <input
                name="console-log"
                type="checkbox"
                checked={props.logBehaviorMessages}
                onClick={() =>
                  props.sendDataToRobot('toggleLogBehaviorMessages')
                }
              />
              <br />
              Talk About&nbsp;
              <input
                name="console-log"
                type="checkbox"
                checked={props.logTalkAboutEvents}
                onClick={() =>
                  props.sendDataToRobot('toggleLogTalkAboutEvents')
                }
              />
              <br />
              Other&nbsp;
              <input
                name="console-log"
                type="checkbox"
                checked={props.logOtherMessages}
                onClick={() => props.sendDataToRobot('toggleLogOtherMessages')}
              />
            </p>
          )}
        </CardBody>
      </Collapse>
    </Card>
  );
};

export default Status;

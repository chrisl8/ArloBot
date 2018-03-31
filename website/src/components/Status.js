import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToYesNo from '../utils/boolToYesNo';
import boolToOnOff from '../utils/boolToOnOff';
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
  if (props.laptopFullyCharged) {
    laptopFullyChargedClass = 'btn btn-success';
    laptopFullyChargedBadgeClass = 'badge badge-light';
  }

  let pluggedInClass = 'btn';
  let pluggedInBadgeClass = 'badge badge-secondary';
  if (props.pluggedIn) {
    pluggedInClass = 'btn btn-danger';
    pluggedInBadgeClass = 'badge badge-light';
  }

  let doorsOpenClass = 'btn';
  let doorsOpenBadgeClass = 'badge badge-secondary';
  if (props.doorsOpen) {
    doorsOpenClass = 'btn btn-danger';
    doorsOpenBadgeClass = 'badge badge-light';
  }

  // let explorePausedClass = 'btn';
  // let explorePausedBadgeClass = 'badge badge-secondary';
  // if (props.explorePaused) {
  //     explorePausedClass = 'btn btn-warning';
  //     explorePausedBadgeClass = 'badge badge-light';
  // }

  let mapClass = 'btn';
  let mapBadgeClass = 'badge badge-secondary';
  let mapBadgeText = 'None';
  if (props.mapName !== '') {
    mapClass = 'btn btn-success';
    mapBadgeClass = 'badge badge-light';
    mapBadgeText = props.mapName;
  }

  // let autoExploreClass = 'btn';
  // let autoExploreBadgeClass = 'badge badge-secondary';
  // if (props.autoExplore) {
  //     autoExploreClass = 'btn btn-warning';
  //     autoExploreBadgeClass = 'badge badge-light';
  // }

  let debuggingClass = 'btn';
  let debuggingBadgeClass = 'badge badge-secondary';
  if (props.debugging) {
    debuggingClass = 'btn btn-warning';
    debuggingBadgeClass = 'badge badge-light';
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

  return (
    <Card id="status-card" className="card-title">
      <CardHeader onClick={() => props.toggle('status')}>
        <CardTitle>Status</CardTitle>
      </CardHeader>
      <Collapse isOpen={props.isOpen}>
        <CardBody>
          <button type="button" className={laptopBatteryClass}>
            Laptop Battery&nbsp;
            <span className="badge badge-light">
              {props.laptopBatteryPercentage}%
            </span>
          </button>
          <button type="button" className={laptopFullyChargedClass}>
            Laptop Fully Charged&nbsp;
            <span className={laptopFullyChargedBadgeClass}>
              {boolToYesNo(props.laptopFullyCharged)}
            </span>
          </button>
          <button
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
            type="button"
            className={doorsOpenClass}
            onClick={() => props.openGroup('startupShutdown')}
          >
            Dangerous Doors Open&nbsp;
            <span className={doorsOpenBadgeClass}>
              {boolToYesNo(props.doorsOpen)}
            </span>
          </button>
          {/* <button type="button" className={explorePausedClass} */}
          {/* onClick={() => props.openGroup('navigation')}>Explore Paused <span */}
          {/* className={explorePausedBadgeClass}>{boolToYesNo(props.explorePaused)}</span> */}
          {/* </button> */}
          <button
            type="button"
            className={mapClass}
            onClick={() => props.openGroup('navigation')}
          >
            Map&nbsp;
            <span className={mapBadgeClass}>{mapBadgeText}</span>
          </button>
          {/* <button type="button" className={autoExploreClass} */}
          {/* onClick={() => props.openGroup('navigation')}>Auto Exploring <span */}
          {/* className={autoExploreBadgeClass}>{boolToYesNo(props.autoExplore)}</span> */}
          {/* </button> */}
          <button
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
            type="button"
            className={masterRelayClass}
            onClick={() => props.sendDataToRobot('toggleMasterRelay')}
          >
            Master Relay&nbsp;
            <span className={masterRelayBadgeClass}>
              {boolToOnOff(props.masterRelayOn)}
            </span>
          </button>
        </CardBody>
      </Collapse>
    </Card>
  );
};

export default Status;

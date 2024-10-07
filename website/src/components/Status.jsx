import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToYesNo from '../utils/boolToYesNo';
import boolToOnOff from '../utils/boolToOnOff';
import boolToUpDown from '../utils/boolToUpDown';
import './Status.css';

const Status = (props) => {
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
          {props.clouServerExists && (
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
          )}
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

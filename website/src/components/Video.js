import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToOnOff from '../utils/boolToOnOff';

const Video = (props) => {
  let cardTitle = 'Video - Camera Off';
  if (props.cameraOn) {
    cardTitle = (
      <span>
        Video&nbsp;-&nbsp;<strong>Camera ON</strong>
      </span>
    );
  }

  let cameraClass = 'btn';
  let cameraBadgeClass = 'badge badge-secondary';
  if (props.cameraOn) {
    cameraClass = 'btn btn-success';
    cameraBadgeClass = 'badge badge-light';
  }

  let { videoSource } = props;
  // For running the web server locally for testing
  if (
    window.location.hostname === 'localhost' &&
    videoSource.substring(0, 7) === 'xscreen'
  ) {
    videoSource = `${props.robotURL}/${props.videoSource}`;
  }

  return (
    <Card id="status-card" className="card-title">
      <CardHeader onClick={() => props.toggle('video')}>
        <CardTitle>{cardTitle}</CardTitle>
      </CardHeader>
      <Collapse isOpen={props.isOpen}>
        <CardBody>
          <button
            id="cameraButton"
            type="button"
            className={cameraClass}
            onClick={() => props.sendDataToRobot('toggleCamera')}
          >
            Camera&nbsp;
            <span className={cameraBadgeClass}>
              {boolToOnOff(props.cameraOn)}
            </span>
          </button>
          <img
            id="videoFeed"
            alt="Video feed"
            src={videoSource}
            style={{ width: '100%' }}
          />
        </CardBody>
      </Collapse>
    </Card>
  );
};

export default Video;

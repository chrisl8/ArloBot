import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToOnOff from '../utils/boolToOnOff';

const Video = (props) => {
  let cardTitle = 'Video - Camera Off';
  if (!props.personalData.camera0) {
    cardTitle = 'Desktop Screenshot';
  }
  if (props.cameraOn) {
    cardTitle = (
      <span>
        Video&nbsp;-&nbsp;<strong>Camera ON</strong>
      </span>
    );
  }

  let cameraClass = 'btn btn-light';
  let cameraBadgeClass = 'badge rounded-pill bg-secondary';
  if (props.cameraOn) {
    cameraClass = 'btn btn-success';
    cameraBadgeClass = 'badge rounded-pill bg-dark';
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
    <Card id="video-card" className="card-title">
      <CardHeader onClick={() => props.toggle('video')}>
        <CardTitle>{cardTitle}</CardTitle>
      </CardHeader>
      <Collapse id="video-card-body" isOpen={props.isOpen}>
        <CardBody>
          {props.personalData.camera0 && (
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
          )}
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

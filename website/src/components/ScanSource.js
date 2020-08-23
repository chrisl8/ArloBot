import React from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

const ScanSource = (props) => {
  const cardTitle = `Scan Source - ${props.personalData.scanTopicSource}`;
  return (
    <Card id="video-card" className="card-title">
      <CardHeader onClick={() => props.toggle('scanSource')}>
        <CardTitle>{cardTitle}</CardTitle>
      </CardHeader>
      <Collapse id="video-card-body" isOpen={props.isOpen}>
        <CardBody>
          {props.personalData.hasASUSXtion && props.personalData.hasKinect && (
            <>
              <strong>Active 3D Camera</strong>
              <br />
              <button
                id="activate-asus_xtion_pro"
                type="button"
                className={
                  props.personalData.active3dCamera === 'asus_xtion_pro'
                    ? 'btn btn-primary'
                    : 'btn btn-default'
                }
              >
                ASUS Xtion
              </button>
              <button
                id="activate-kinect"
                type="button"
                className={
                  props.personalData.active3dCamera === 'kinect'
                    ? 'btn btn-primary'
                    : 'btn btn-default'
                }
              >
                Kinect
              </button>
              <hr />
            </>
          )}
          <strong>Scan Topic Source</strong>
          <br />
          {props.personalData.hasASUSXtion && (
            <button
              id="asus_xtion_pro"
              type="button"
              className={
                props.personalData.scanTopicSource === 'asus_xtion_pro'
                  ? 'btn btn-primary'
                  : 'btn btn-default'
              }
            >
              ASUS Xtion
            </button>
          )}
          {props.personalData.hasKinect && (
            <button
              id="kinect"
              type="button"
              className={
                props.personalData.scanTopicSource === 'kinect'
                  ? 'btn btn-primary'
                  : 'btn btn-default'
              }
            >
              Kinect
            </button>
          )}
          {props.personalData.hasXV11 && (
            <button
              id="xv11"
              type="button"
              className={
                props.personalData.scanTopicSource === 'xv11'
                  ? 'btn btn-primary'
                  : 'btn btn-default'
              }
            >
              XV11 Neato
            </button>
          )}
          {props.personalData.hasScanseSweep && (
            <button
              id="scanse"
              type="button"
              className={
                props.personalData.scanTopicSource === 'scanse'
                  ? 'btn btn-primary'
                  : 'btn btn-default'
              }
            >
              Scanse Sweep
            </button>
          )}
          {props.personalData.hasRPLIDAR && (
            <button
              id="rplidar"
              type="button"
              className={
                props.personalData.scanTopicSource === 'rplidar'
                  ? 'btn btn-primary'
                  : 'btn btn-default'
              }
            >
              RPLIDAR
            </button>
          )}
        </CardBody>
      </Collapse>
    </Card>
  );
};

export default ScanSource;

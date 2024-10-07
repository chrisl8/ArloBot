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
          <strong>Scan Topic Source</strong>
          <br />
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

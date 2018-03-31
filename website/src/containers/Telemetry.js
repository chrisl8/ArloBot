import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

class Telemetry extends Component {
  constructor(props) {
    super(props);
    this.toggle = this.toggle.bind(this);
    this.state = { isOpen: false };
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  render() {
    let cardBody = <p>These do not work until ROS is Running.</p>;
    if (this.props.ROSisRunning) {
      cardBody = this.props.rosTopicItems.map((entry) => {
        let thisButtonClass = 'btn';
        let thisButtonBadgeClass = 'badge badge-secondary';
        if (entry.btnClass !== '') {
          thisButtonClass = `btn ${entry.btnClass}`;
          thisButtonBadgeClass = 'badge badge-light';
        }
        return (
          <span key={entry.rosName}>
            <button type="button" className={thisButtonClass}>
              {entry.fancyName}&nbsp;
              <span className={thisButtonBadgeClass}>{`${entry.status}`}</span>
            </button>
          </span>
        );
      });
    }
    return (
      <Card id="status-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Telemetry</CardTitle>
        </CardHeader>
        <Collapse isOpen={this.state.isOpen}>
          <CardBody>{cardBody}</CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default Telemetry;

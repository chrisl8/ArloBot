import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToOnOff from '../utils/boolToOnOff';

class Relays extends Component {
  constructor(props) {
    super(props);
    this.toggle = this.toggle.bind(this);
    this.state = { isOpen: false };
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  render() {
    const cardBody = this.props.relays.map((entry) => {
      let thisButtonClass = 'btn';
      let thisButtonBadgeClass = 'badge badge-secondary';
      if (entry.relayOn) {
        thisButtonClass = 'btn btn-success';
        thisButtonBadgeClass = 'badge badge-light';
      }
      return (
        <span key={entry.number}>
          <button
            type="button"
            className={thisButtonClass}
            onClick={() =>
              this.props.sendDataToRobot('toggleRelay', entry.number)
            }
          >
            {entry.fancyName}&nbsp;
            <span className={thisButtonBadgeClass}>
              {boolToOnOff(entry.relayOn)}
            </span>
          </button>
        </span>
      );
    });
    return (
      <Card id="status-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Relays</CardTitle>
        </CardHeader>
        <Collapse isOpen={this.state.isOpen}>
          <CardBody>{cardBody}</CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default Relays;

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
    const buttonIdList = [];
    const cardBody = this.props.relays.map((entry) => {
      let buttonId = `${entry.name}RelayButton`;
      if (buttonIdList.indexOf(buttonId) > -1) {
        buttonId = `${buttonId}${entry.number}`;
      }
      buttonIdList.push(buttonId);
      let thisButtonClass = 'btn';
      let thisButtonBadgeClass = 'badge badge-secondary';
      if (entry.relayOn) {
        thisButtonClass = 'btn btn-success';
        thisButtonBadgeClass = 'badge badge-light';
      }
      return (
        <span key={entry.number}>
          <button
            id={buttonId}
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
      <Card id="relays-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Relays</CardTitle>
        </CardHeader>
        <Collapse id="relays-card-body" isOpen={this.state.isOpen}>
          <CardBody>
            {!this.props.masterRelayOn && (
              <p>Relays will not function while Master Relay is off.</p>
            )}
            {cardBody}
          </CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default Relays;

import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';
import boolToOnOff from '../utils/boolToOnOff';

class Behavior extends Component {
  constructor(props) {
    super(props);
    this.state = {
      isOpen: false,
      textToSpeak: '',
      askRobot: '',
    };

    this.handleTextToSpeakChange = this.handleTextToSpeakChange.bind(this);
    this.sendTextToSpeak = this.sendTextToSpeak.bind(this);
    this.handleAskRobotTextChange = this.handleAskRobotTextChange.bind(this);
    this.askRobot = this.askRobot.bind(this);
    this.toggle = this.toggle.bind(this);
  }

  handleTextToSpeakChange(event) {
    this.setState({ textToSpeak: event.target.value });
  }

  handleAskRobotTextChange(event) {
    this.setState({ askRobot: event.target.value });
  }

  sendTextToSpeak(event) {
    this.props.sendDataToRobot('tts', this.state.textToSpeak);
    event.preventDefault();
  }

  toggle() {
    this.setState({ isOpen: !this.state.isOpen });
  }

  askRobot(event) {
    this.props.sendDataToRobot('ask', this.state.askRobot);
    event.preventDefault();
  }

  render() {
    let idleButtonClass = 'toggle';
    let idleButtonLeftSideLabelClass = 'toggler';
    let idleButtonRightSideLabelClass = 'toggler';
    let idleButtonValueToSend = 'stopIdleTimer';
    if (this.props.idleTimeout) {
      idleButtonClass += ' toggle-on';
      idleButtonRightSideLabelClass += ' brightly-negative-text';
    } else {
      idleButtonValueToSend = 'startIdleTimer';
      idleButtonClass += ' toggle-off';
      idleButtonLeftSideLabelClass += ' brightly-positive-text';
    }

    let talkButtonClass = 'toggle';
    let talkButtonLeftSideLabelClass = 'toggler';
    let talkButtonRightSideLabelClass = 'toggler';
    let talkButtonValueToSend = 'talk';
    if (this.props.beQuiet) {
      talkButtonClass += ' toggle-on';
      talkButtonRightSideLabelClass += ' brightly-negative-text';
    } else {
      talkButtonValueToSend = 'beQuiet';
      talkButtonClass += ' toggle-off';
      talkButtonLeftSideLabelClass += ' brightly-positive-text';
    }

    let blinkyLightButtonClass = 'btn btn-light';
    if (this.props.neoPixelsOn) {
      blinkyLightButtonClass = 'btn btn-warning';
    }

    return (
      <Card id="behavior-card" className="card-title">
        <CardHeader onClick={this.toggle}>
          <CardTitle>Behavior</CardTitle>
        </CardHeader>
        <Collapse id="behavior-card-body" isOpen={this.state.isOpen}>
          {/* TODO: Can these spread and wrap responsively? */}
          <CardBody>
            <div className="flex-column-wrap">
              <div className="flex-row-wrap">
                <div className="no-flex">
                  <form
                    id="saySomethingForm"
                    name="saySomethingForm"
                    onSubmit={this.sendTextToSpeak}
                  >
                    <input
                      type="text"
                      id="say-something"
                      placeholder="What should I say?"
                      value={this.state.textToSpeak}
                      onChange={this.handleTextToSpeakChange}
                    />
                    <input
                      type="submit"
                      value="Speak"
                      className="btn btn-primary"
                    />
                  </form>
                </div>
              </div>
              <div className="flex-row-wrap">
                <div
                  className="lcarish-toggle-button no-flex"
                  id="idle-timeout-button"
                  onClick={() =>
                    this.props.sendDataToRobot(idleButtonValueToSend)
                  }
                >
                  {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
                  <label
                    htmlFor="idle-timeout-switch"
                    className={idleButtonLeftSideLabelClass}
                  >
                    Never
                  </label>
                  <div className={idleButtonClass}>
                    <input
                      id="idle-timeout-switch"
                      type="checkbox"
                      className="check"
                      checked={this.props.idleTimeout}
                      readOnly
                    />
                    <span className="b switch">Idle</span>
                  </div>
                  {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
                  <label className={idleButtonRightSideLabelClass}>
                    Timeout
                  </label>
                </div>
                <div
                  className="lcarish-toggle-button no-flex"
                  id="talk-bequiet-button"
                  onClick={() =>
                    this.props.sendDataToRobot(talkButtonValueToSend)
                  }
                >
                  {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
                  <label className={talkButtonLeftSideLabelClass}>Talk</label>
                  <div className={talkButtonClass}>
                    <input
                      type="checkbox"
                      className="check"
                      checked={this.props.beQuiet}
                      readOnly
                    />
                    <span className="b switch">Sound</span>
                  </div>
                  {/* eslint-disable-next-line jsx-a11y/label-has-associated-control */}
                  <label className={talkButtonRightSideLabelClass}>Quiet</label>
                </div>
              </div>
              {this.props.useArduinoForBlinkenLights && (
                <div className="flex-row-wrap behavior-buttons">
                  <button
                    id="blinky-lights-button"
                    type="button"
                    className={blinkyLightButtonClass}
                    onClick={() => this.props.sendDataToRobot('arduino')}
                  >
                    Blinky Lights
                  </button>
                </div>
              )}
            </div>
          </CardBody>
        </Collapse>
      </Card>
    );
  }
}

export default Behavior;

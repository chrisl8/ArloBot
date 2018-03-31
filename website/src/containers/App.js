import React, { Component } from 'react';
import openSocket from 'socket.io-client';
import './App.css';
import AccordionGroup from './AccordionGroup';

import Banner from '../components/Banner';

class App extends Component {
  constructor(props) {
    super(props);
    this.state = {
      webModel: {
        status: 'Robot Offline',
      },
    };
    this.sendDataToRobot = this.sendDataToRobot.bind(this);
  }

  componentDidMount() {
    // this.socket = openSocket(`http://${window.location.hostname}:8080`); // For testing
    this.socket = openSocket();

    this.socket.on('startup', (data) => {
      this.setState({
        webModel: data,
      });
    });

    this.socket.on('webModel', (data) => {
      this.setState({
        webModel: data,
      });
    });
    this.socket.on('disconnect', () => {
      this.setState({
        webModel: {
          status: 'Robot Offline',
        },
      });
    });
  }

  sendDataToRobot(value, data) {
    if (this.socket) {
      if (data !== undefined) {
        this.socket.emit(value, data);
      } else {
        this.socket.emit(value);
      }
    }
  }

  render() {
    return (
      <div className="decorative-border">
        <Banner
          haltRobot={this.state.webModel.haltRobot}
          status={this.state.webModel.status}
          behaviorStatus={this.state.webModel.behaviorStatus}
          sendDataToRobot={this.sendDataToRobot}
        />
        {/* Doing if/else in REACT https://www.robinwieruch.de/conditional-rendering-react/ */}
        {this.state.webModel.status !== 'Robot Offline' && (
          <AccordionGroup
            laptopBatteryPercentage={
              this.state.webModel.laptopBatteryPercentage
            }
            laptopFullyCharged={this.state.webModel.laptopFullyCharged}
            pluggedIn={this.state.webModel.pluggedIn}
            doorsOpen={this.state.webModel.doorsOpen}
            explorePaused={this.state.webModel.rosParameters.explorePaused}
            mapName={this.state.webModel.mapName}
            autoExplore={this.state.webModel.autoExplore}
            debugging={this.state.webModel.debugging}
            cameraOn={this.state.webModel.cameraOn}
            videoSource={this.state.webModel.videoSource}
            masterRelayOn={this.state.webModel.masterRelayOn}
            ROSisRunning={this.state.webModel.ROSisRunning}
            rosTopicItems={this.state.webModel.rosTopicItems}
            relays={this.state.webModel.relays}
            idleTimeout={this.state.webModel.idleTimeout}
            beQuiet={this.state.webModel.beQuiet}
            myCroftSaid={this.state.webModel.myCroftSaid}
            colorFollowerRunning={this.state.webModel.colorFollowerRunning}
            neoPixelsOn={this.state.webModel.neoPixelsOn}
            ROSstart={this.state.webModel.ROSstart}
            monitorACconnection={
              !this.state.webModel.rosParameters.monitorACconnection
            }
            ignoreIRSensors={this.state.webModel.rosParameters.ignoreIRSensors}
            ignoreCliffSensors={
              this.state.webModel.rosParameters.ignoreCliffSensors
            }
            ignoreFloorSensors={
              this.state.webModel.rosParameters.ignoreFloorSensors
            }
            ignoreProximity={this.state.webModel.rosParameters.ignoreProximity}
            makeMap={this.state.webModel.makeMap}
            makeMapRunning={this.state.webModel.makeMapRunning}
            mapList={this.state.webModel.mapList}
            wayPoints={this.state.webModel.wayPoints}
            unplugYourself={this.state.webModel.unplugYourself}
            scrollingStatus={this.state.webModel.scrollingStatus}
            logStreamerRunning={this.state.webModel.logStreamerRunning}
            RosService={this.RosService}
            sendDataToRobot={this.sendDataToRobot}
          />
        )}
        <p id="footer-line">
          <a href="https://github.com/chrisl8/ArloBot">
            https://github.com/chrisl8/ArloBot
          </a>
        </p>
      </div>
    );
  }
}

export default App;

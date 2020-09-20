import React, { Component } from 'react';
import openSocket from 'socket.io-client';
import './App.css';
import AccordionGroup from './AccordionGroup';
import Settings from './PersonalSettings';

import Banner from '../components/Banner';

class App extends Component {
  constructor(props) {
    super(props);
    this.state = {
      webModel: {
        status: 'Robot Offline',
      },
      personalData: {},
    };
    this.sendDataToRobot = this.sendDataToRobot.bind(this);
  }

  componentDidMount() {
    // For local testing on the robot:
    // this.socket = openSocket(`http://${window.location.hostname}:8080`);
    // For testing running locally while connecting site to a remote robot:
    // this.socket = openSocket(`http://twoflower.local:8080`);
    // For production:
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

    this.socket.on('personalData', (data) => {
      this.setState({
        personalData: data,
      });
    });

    this.socket.on('disconnect', () => {
      this.setState({
        webModel: {
          status: 'Robot Offline',
        },
      });
    });

    // Populate personal data on startup
    this.sendDataToRobot('getPersonalData');
  }

  handleBottomButton = (e) => {
    this.sendDataToRobot('getPersonalData');
    this.setState({ appFunction: e.target.id });
  };

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
    let pageContent = <></>;
    let bottomButton = <></>;
    if (this.state.webModel.status !== 'Robot Offline') {
      if (this.state.appFunction === 'settings') {
        pageContent = <Settings personalData={this.state.personalData} />;
        bottomButton = (
          <button
            type="button"
            id="control"
            className="btn btn-primary settings-button"
            onClick={this.handleBottomButton}
          >
            Control
          </button>
        );
      } else {
        pageContent = (
          <AccordionGroup
            personalData={this.state.personalData}
            laptopBatteryPercentage={
              this.state.webModel.laptopBatteryPercentage
            }
            laptopFullyCharged={this.state.webModel.laptopFullyCharged}
            pluggedIn={this.state.webModel.pluggedIn}
            mapName={this.state.webModel.mapName}
            mapLoaded={this.state.webModel.mapLoaded}
            debugging={this.state.webModel.debugging}
            logConsoleMessages={this.state.webModel.logConsoleMessages}
            logBehaviorMessages={this.state.webModel.logBehaviorMessages}
            logOtherMessages={this.state.webModel.logOtherMessages}
            cameraOn={this.state.webModel.cameraOn}
            videoSource={this.state.webModel.videoSource}
            masterRelayOn={this.state.webModel.masterRelayOn}
            myCroftIsRunning={this.state.webModel.myCroftIsRunning}
            cloudServerConnected={this.state.webModel.cloudServerConnected}
            ROSisRunning={this.state.webModel.ROSisRunning}
            rosTopicItems={this.state.webModel.rosTopicItems}
            relays={this.state.webModel.relays}
            idleTimeout={this.state.webModel.idleTimeout}
            beQuiet={this.state.webModel.beQuiet}
            myCroftSaid={this.state.webModel.myCroftSaid}
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
            navigationInProgress={this.state.webModel.navigationInProgress}
            lastNavigationResult={this.state.webModel.lastNavigationResult}
            wayPointName={this.state.webModel.wayPointNavigator.wayPointName}
            unplugYourself={this.state.webModel.unplugYourself}
            scrollingStatus={this.state.webModel.scrollingStatus}
            logStreamerRunning={this.state.webModel.logStreamerRunning}
            RosService={this.RosService}
            robotURL={this.state.webModel.robotURL}
            sendDataToRobot={this.sendDataToRobot}
          />
        );
        bottomButton = (
          <button
            type="button"
            id="settings"
            className="btn btn-primary settings-button"
            onClick={this.handleBottomButton}
          >
            Settings
          </button>
        );
      }
    }
    return (
      <div className="decorative-border">
        <Banner
          haltRobot={this.state.webModel.haltRobot}
          status={this.state.webModel.status}
          behaviorStatus={this.state.webModel.behaviorStatus}
          sendDataToRobot={this.sendDataToRobot}
        />
        {pageContent}
        <p id="footer-line">
          <a href="https://github.com/chrisl8/ArloBot">
            https://github.com/chrisl8/ArloBot
          </a>
          {bottomButton}
        </p>
      </div>
    );
  }
}

export default App;

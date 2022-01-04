import React, { Component } from 'react';
import Status from '../components/Status';
import Telemetry from './Telemetry';
import Relays from './Relays';
import Behavior from './Behavior';
import Sensors from './Sensors';
import StartupShutdown from './StartupShutdown';
import RobotServiceLog from './RobotServiceLog';
import Navigation from './Navigation';
import RemoteControl from './RemoteControl';
import Video from '../components/Video';
import ScanSource from '../components/ScanSource';

class AccordionGroup extends Component {
  constructor(props) {
    super(props);
    this.state = {
      statusIsOpen: true,
      startupShutdownIsOpen: true,
      navigationIsOpen: false,
      videoIsOpen: false,
    };
    // Not all groups need external access to their open/closed state.
    // Add new ones as needed.
    this.toggle = this.toggle.bind(this);
    this.openGroup = this.openGroup.bind(this);
  }

  // Ref usage
  // https://stackoverflow.com/questions/43441856/reactjs-how-to-scroll-to-an-element If you get an
  // error about attaching an ref to a stateless functional component,
  // remember it is about the element you attach it to, not the container you do it in!
  handleUpdateStartupElement = (element) => {
    this.startupElement = element;
  };

  handleUpdateNavigationElement = (element) => {
    this.navigationElement = element;
  };

  handleScrollToStart() {
    if (this.startupElement) {
      // console.log(this.startupElement);
      // NOTE: you can't put these refs on just anything. I made a wrapping <div> for it,
      // as I was getting weird "scrollIntoView" is not a function and
      // cannot attach ref to stateless functional components
      // errors otherwise.
      // I think you can also stick .parentNode on the end of the ref if you want to walk up the DOM
      this.startupElement.scrollIntoView();
      // Works in Chrome and latest FF, See MDN for more options.
    }
  }

  handleScrollToNavigation() {
    if (this.navigationElement) {
      this.navigationElement.scrollIntoView();
    }
  }

  toggle(group) {
    const newState = {};
    newState[`${group}IsOpen`] = !this.state[`${group}IsOpen`];
    this.setState(newState);
  }

  openGroup(group) {
    const newState = {};
    newState[`${group}IsOpen`] = true;
    this.setState(newState);
    // TODO: This is always scrolling to start, but sometimes we opened a different group.
    if (group === 'startupShutdown') {
      this.handleScrollToStart();
    } else if (group === 'navigation') {
      this.handleScrollToNavigation();
    }
  }

  render() {
    const displayScanSource =
      [
        this.props.personalData.hasASUSXtion,
        this.props.personalData.hasKinect,
        this.props.personalData.hasXV11,
        this.props.personalData.hasScanseSweep,
        this.props.personalData.hasRPLIDAR,
      ].filter((entry) => entry).length > 1;
    return (
      <>
        <Status
          laptopBatteryPercentage={this.props.laptopBatteryPercentage}
          laptopFullyCharged={this.props.laptopFullyCharged}
          pluggedIn={this.props.pluggedIn}
          mapName={this.props.mapName}
          debugging={this.props.debugging}
          logConsoleMessages={this.props.logConsoleMessages}
          isOpen={this.state.statusIsOpen}
          toggle={this.toggle}
          openGroup={this.openGroup}
          cameraOn={this.props.cameraOn}
          useMasterPowerRelay={this.props.useMasterPowerRelay}
          masterRelayOn={this.props.masterRelayOn}
          useMyCroft={this.props.useMyCroft}
          myCroftIsRunning={this.props.myCroftIsRunning}
          clouServerExists={this.props.clouServerExists}
          cloudServerConnected={this.props.cloudServerConnected}
          sendDataToRobot={this.props.sendDataToRobot}
          personalData={this.props.personalData}
        />
        {this.props.ROSisRunning && (
          <Telemetry
            ROSisRunning={this.props.ROSisRunning}
            rosTopicItems={this.props.rosTopicItems}
          />
        )}
        {this.props.useUSBrelay && (
          <Relays
            relays={this.props.relays}
            masterRelayOn={this.props.masterRelayOn}
            sendDataToRobot={this.props.sendDataToRobot}
          />
        )}
        <Behavior
          idleTimeout={this.props.idleTimeout}
          sendDataToRobot={this.props.sendDataToRobot}
          beQuiet={this.props.beQuiet}
          myCroftSaid={this.props.myCroftSaid}
          ROSisRunning={this.props.ROSisRunning}
          useArduinoForBlinkenLights={this.props.useArduinoForBlinkenLights}
          neoPixelsOn={this.props.neoPixelsOn}
        />
        {this.props.ROSisRunning && (
          <Sensors
            ROSisRunning={this.props.ROSisRunning}
            monitorACconnection={this.props.monitorACconnection}
            ignoreIRSensors={this.props.ignoreIRSensors}
            ignoreCliffSensors={this.props.ignoreCliffSensors}
            ignoreFloorSensors={this.props.ignoreFloorSensors}
            ignoreProximity={this.props.ignoreProximity}
            sendDataToRobot={this.props.sendDataToRobot}
          />
        )}
        <StartupShutdown
          isOpen={this.state.startupShutdownIsOpen}
          toggle={this.toggle}
          ROSisRunning={this.props.ROSisRunning}
          ROSstart={this.props.ROSstart}
          unplugYourself={this.props.unplugYourself}
          pluggedIn={this.props.pluggedIn}
          sendDataToRobot={this.props.sendDataToRobot}
          handleUpdateStartupElement={this.handleUpdateStartupElement}
        />
        {displayScanSource && (
          <ScanSource
            isOpen={this.state.scanSourceIsOpen}
            toggle={this.toggle}
            personalData={this.props.personalData}
          />
        )}
        <RobotServiceLog
          scrollingStatus={this.props.scrollingStatus}
          logStreamerRunning={this.props.logStreamerRunning}
          ROSisRunning={this.props.ROSisRunning}
          sendDataToRobot={this.props.sendDataToRobot}
        />
        {this.props.ROSisRunning && (
          <Navigation
            isOpen={this.state.navigationIsOpen}
            toggle={this.toggle}
            makeMap={this.props.makeMap}
            makeMapRunning={this.props.makeMapRunning}
            mapName={this.props.mapName}
            mapLoaded={this.props.mapLoaded}
            mapList={this.props.mapList}
            wayPoints={this.props.wayPoints}
            sendDataToRobot={this.props.sendDataToRobot}
            pluggedIn={this.props.pluggedIn}
            navigationInProgress={this.props.navigationInProgress}
            lastNavigationResult={this.props.lastNavigationResult}
            wayPointName={this.props.wayPointName}
            handleUpdateNavigationElement={this.handleUpdateNavigationElement}
          />
        )}
        {this.props.ROSisRunning && (
          <RemoteControl
            ROSisRunning={this.props.ROSisRunning}
            openGroup={this.openGroup}
            RosService={this.props.RosService}
            pluggedIn={this.props.pluggedIn}
            sendDataToRobot={this.props.sendDataToRobot}
          />
        )}
        {Boolean(this.props.thisSystemHasAScreen || this.props.camera0) && (
          <Video
            isOpen={this.state.videoIsOpen}
            toggle={this.toggle}
            cameraOn={this.props.cameraOn}
            videoSource={this.props.videoSource}
            robotURL={this.props.robotURL}
            sendDataToRobot={this.props.sendDataToRobot}
            personalData={this.props.personalData}
          />
        )}
      </>
    );
  }
}

export default AccordionGroup;

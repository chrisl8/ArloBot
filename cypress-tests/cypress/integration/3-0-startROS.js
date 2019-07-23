import {
  resetRobotService,
  initialPageLoadItemsVisible,
  openPanelIfClosed
} from "../support/reusableTestsAndSetupTasks";

import { startROS, stopROS } from "../support/rosStartStop";

import { additionalItemsAreVisibleWhenRosIsRunning } from "../support/panelTestsWithRosRunning";

import { robotServiceLogPanelShouldBeOpen } from "../support/panelTestsWithRosOff";

describe("Start ROS", () => {
  resetRobotService();
  initialPageLoadItemsVisible();
  openPanelIfClosed("robot-service-log");
  robotServiceLogPanelShouldBeOpen(true);
  startROS();
  additionalItemsAreVisibleWhenRosIsRunning();

  it("Master Relay should be on", () => {
    cy.get("#masterRelayStatusButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");
  });

  openPanelIfClosed("telemetry");

  // Telemetry
  it("Telemetry Panel should exist and contain correct data", () => {
    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .contains("span", "false", { timeout: 30000 })
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .contains("span", "false")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Safe To Recede")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Safe To Recede")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Safe To Recede")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Safe To Proceed")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Safe To Proceed")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Safe To Proceed")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Escaping")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Escaping")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Escaping")
      .contains("span", "false")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Min Distance Sensor")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Min Distance Sensor")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Min Distance Sensor")
      .children("span")
      .contains(/[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .children("span")
      .contains(/[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .children("span")
      .contains(/[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Heading")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Heading")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Heading")
      .children("span")
      .contains(/[0-9]*\.[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Gyro Heading")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Gyro Heading")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Gyro Heading")
      .children("span")
      .contains(/[0-9]*\.[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Left Motor Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Left Motor Power")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Left Motor Power")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Right Motor Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Right Motor Power")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Right Motor Power")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Laptop Battery")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Laptop Battery")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Laptop Battery")
      .children("span")
      .contains(/[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Robot Battery Volts")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Robot Battery Volts")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Robot Battery Volts")
      .children("span")
      .contains(/[0-9]*\.[0-9]*/)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Robot Battery Low")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Robot Battery Low")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Robot Battery Low")
      .contains("span", "false")
      .should("be.visible");
  });

  openPanelIfClosed("relays");

  // Relays
  // Five volt should be on
  it("Relay Panel Five Volt Relay should be on", () => {
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .should("have.class", "btn-success");
  });

  openPanelIfClosed("behavior");

  it("Behavior Panel should contain new items when ROS is running", () => {
    cy.get("#behavior-card")
      .get("#start-color-follower-button")
      .should("be.visible");
  });

  openPanelIfClosed("sensors");

  // Sensors
  it("Sensors Panel should contain correct switches, all off", () => {
    cy.get("#monitorACconnection")
      .contains("AC")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreIRSensors")
      .contains("Infrared")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreCliffSensors")
      .contains("Cliff")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreFloorSensors")
      .contains("Floor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreProximity")
      .contains("PING")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");
  });

  // Test Monitor AC while plugged in
  it("test disable Monitor AC while plugged in", () => {
    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .contains("span", "true")
      .should("be.visible");

    cy.get("#monitorACconnection")
      .contains("AC")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#monitorACconnection").click();

    cy.get("#monitorACconnection")
      .contains("AC")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("not.have.class", "brightly-positive-text");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Ignore")
      .should("have.class", "brightly-negative-text");

    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .contains("span", "false")
      .should("be.visible");

    cy.get("#monitorACconnection").click();

    cy.get("#monitorACconnection")
      .contains("AC")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#monitorACconnection")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .should("have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("AC Power")
      .contains("span", "true")
      .should("be.visible");
  });

  it("shut off Monitor PING and IR Sensors", () => {
    cy.get("#ignoreIRSensors")
      .contains("Infrared")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreIRSensors").click();

    cy.get("#ignoreIRSensors")
      .contains("Infrared")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("not.have.class", "brightly-positive-text");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Ignore")
      .should("have.class", "brightly-negative-text");

    cy.get("#ignoreProximity")
      .contains("PING")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreProximity").click();

    cy.get("#ignoreProximity")
      .contains("PING")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("not.have.class", "brightly-positive-text");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Ignore")
      .should("have.class", "brightly-negative-text");

    // With IR and Proximity sensors off,
    // Forward and Reverse speed limit should be at max.
    // TODO: Check other things that should be set
    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Forward Speed Limit")
      .children("span")
      .contains(220)
      .should("be.visible");

    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Reverse Speed Limit")
      .children("span")
      .contains(220)
      .should("be.visible");

    cy.get("#ignoreIRSensors").click();

    cy.get("#ignoreIRSensors")
      .contains("Infrared")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreIRSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreProximity").click();

    cy.get("#ignoreProximity")
      .contains("PING")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreProximity")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");
  });

  it("shut off Cliff Sensors", () => {
    cy.get("#ignoreCliffSensors")
      .contains("Cliff")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreCliffSensors").click();

    cy.get("#ignoreCliffSensors")
      .contains("Cliff")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("not.have.class", "brightly-positive-text");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Ignore")
      .should("have.class", "brightly-negative-text");

    // With Cliff Sensor Off there can be no Cliff Detected

    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Cliff Detected")
      .contains("span", "false")
      .should("be.visible");

    cy.get("#ignoreCliffSensors").click();

    cy.get("#ignoreCliffSensors")
      .contains("Cliff")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreCliffSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");
  });

  it("shut off Floor Sensors", () => {
    cy.get("#ignoreFloorSensors")
      .contains("Floor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");

    cy.get("#ignoreFloorSensors").click();

    cy.get("#ignoreFloorSensors")
      .contains("Floor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("not.have.class", "brightly-positive-text");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Ignore")
      .should("have.class", "brightly-negative-text");

    // With Floor Sensor Off there can be no Floor Obstacle Detected

    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .should("be.visible");
    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .should("not.have.class", "btn-danger");
    cy.get("#telemetry-card")
      .contains("Floor Obstacle")
      .contains("span", "false")
      .should("be.visible");

    cy.get("#ignoreFloorSensors").click();

    cy.get("#ignoreFloorSensors")
      .contains("Floor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor")
      .should("be.visible");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Monitor", { timeout: 10000 })
      .should("have.class", "brightly-positive-text");
    cy.get("#ignoreFloorSensors")
      .children("label")
      .contains("Ignore")
      .should("not.have.class", "brightly-negative-text");
  });

  // Startup/Shutdown
  it("Startup/Shutdown contents with ROS Running", () => {
    cy.get("#startup-shutdown-card")
      .get("#set-all-doors-closed-button")
      .contains("Set All Doors Closed")
      .should("be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .should("not.be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Starting...")
      .should("not.be.visible");

    cy.get("#startup-shutdown-card")
      .contains("ROS is Running.")
      .should("be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Stop ROS")
      .should("be.visible");

    cy.get("#startup-shutdown-card")
      .get("#unplug-yourself-button")
      .contains("Unplug")
      .should("be.visible");
    cy.get("#startup-shutdown-card")
      .get("#unplug-yourself-button")
      .children("label")
      .contains("No")
      .should("be.visible");
    cy.get("#startup-shutdown-card")
      .get("#unplug-yourself-button")
      .children("label")
      .contains("No")
      .should("have.class", "brightly-positive-text");
    cy.get("#startup-shutdown-card")
      .get("#unplug-yourself-button")
      .children("label")
      .contains("Yes")
      .should("not.have.class", "brightly-negative-text");
  });

  // Robot Service Log
  openPanelIfClosed("robot-service-log");
  // TODO: Check contents
  // TODO: Test Log Streamer

  // Navigation
  openPanelIfClosed("navigation");
  // TODO: Check contents

  // Remote Control
  openPanelIfClosed("remote-control");
  // TODO: Check contents

  stopROS();

  it("Relay Panel Five Volt Relay should be Off", () => {
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .should("not.have.class", "btn-success");
  });

  it("Master Relay should be Off", () => {
    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  });
});

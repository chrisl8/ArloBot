import { openPanelIfClosed } from "./reusableTestsAndSetupTasks";

function startROS() {
  openPanelIfClosed("robot-service-log");
  it("Start ROS", () => {
    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .click();

    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .should("not.be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Starting...")
      .should("be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Please wait, ROS is starting . . .")
      .should("be.visible");

    // Log messages:
    cy.get("#statusScrollBox")
      .contains("ROS is starting up . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Start ROS: Process starting!")
      .should("be.visible");
    // cy.contains("Clearing ROS Logs . . .").should("be.visible");
    // Clearing ROS Logs only happens if there ARE ROS Logs, so not consistent.
    cy.get("#statusScrollBox")
      .contains("Turning on Arlo Power supply . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("relay on 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Giving it 1 second to come online . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Master Relay on", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Turning on Five Volt power converter . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Camera 0 . . .", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Xbox Controller . . .", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Activity Board . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Quick Start Board . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Hardware Check SUCCESS! All devices found.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Waiting for roscore to start . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Use kill", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ros.sh to close.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Rosapi started", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROS successfully started.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket connected.")
      .should("be.visible");
  });
}

function checkInitialTelemetry() {
  openPanelIfClosed("telemetry");
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

    // IR Sensors are Ignored by Default.
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
    // IR Sensors are Ignored by Default.
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

  it("test enable/disable Monitor IR Sensors", () => {
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
}

function stopROS() {
  openPanelIfClosed("robot-service-log");
  it("Stop ROS", () => {
    cy.get("#startup-shutdown-card")
      .contains("Stop ROS")
      .click();

    // Log messages:
    cy.get("#statusScrollBox")
      .contains("ROS is starting up . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Running kill_ros.sh . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Killing everything, please wait...")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown:")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Waiting for Robot to close . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket closed")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Waiting for Robot to close . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Waiting for roscore to close . . .", {
        timeout: 10000
      })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROS exited with code: 143")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Start ROS: Process Closed.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Everything Killed.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Turning off all relays")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Turning off Arlo Power.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: relay off 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("kill_ros.sh closed with code 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Master Relay off", { timeout: 30000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket closed")
      .should("be.visible");

    cy.get("#navigation-card").should("not.be.visible");
    cy.get("#remote-control-card").should("not.be.visible");
    cy.get("#telemetry-card").should("not.be.visible");
    cy.get("#sensors-card").should("not.be.visible");

    cy.contains("ROS process has closed.").should("be.visible");
    cy.contains("Waiting for StartROS request.").should("be.visible");
  });
}

function checkPostRosShutdownStatus() {
  it("Relay Panel Five Volt Relay should be Off", () => {
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .contains("span", "Off", { timeout: 10000 })
      .should("be.visible");
    cy.get("#relays-card")
      .get("#fiveVoltRelayButton")
      .should("not.have.class", "btn-success");
  });

  it("Master Relay should be Off", () => {
    cy.get("#masterRelayStatusButton")
      .contains("span", "Off", { timeout: 15000 })
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  });
}

module.exports = { startROS, checkInitialTelemetry, stopROS, checkPostRosShutdownStatus };

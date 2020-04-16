import {
  resetRobotService,
  initialPageLoadItemsVisible,
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import { startROS, checkInitialTelemetry, stopROS, checkPostRosShutdownStatus } from "../support/rosStartStop";

import panelTestsWithRosRunning, {
  additionalItemsAreVisibleWhenRosIsRunning
} from "../support/panelTestsWithRosRunning";

import panelTestsWithRosOff from "../support/panelTestsWithRosOff";

describe("Start and Stop ROS", () => {
  resetRobotService();
  initialPageLoadItemsVisible();
  openPanelIfClosed("robot-service-log");
  panelTestsWithRosOff.robotServiceLogPanelShouldBeOpen(true);
  startROS();
  additionalItemsAreVisibleWhenRosIsRunning();

  it("Master Relay should be on", () => {
    cy.get("#masterRelayStatusButton")
      .contains("span", "On", { timeout: 15000 })
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");
  });

  checkInitialTelemetry();

  // Robot Service Log
  openPanelIfClosed("robot-service-log");
  panelTestsWithRosRunning.robotServiceLogPanelShouldBeOpen(true);
  it("Robot Log Streamer should work", () => {
    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .should("not.have.class", "btn-success");

    cy.get("#robot-service-log-card")
      .get("#view-log-streamer-button")
      .should("not.be.visible");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "Off")
      .click();

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .should("have.class", "btn-success");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "On")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#view-log-streamer-button")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#view-log-streamer-button")
      .click();

    cy.wait(30000);

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "On")
      .click();

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .should("not.have.class", "btn-success");

    cy.get("#robot-service-log-card")
      .get("#view-log-streamer-button")
      .should("not.be.visible");
  });
  closePanelIfOpen("robot-service-log");
  panelTestsWithRosRunning.robotServiceLogPanelShouldBeOpen(false);

  // Navigation
  openPanelIfClosed("navigation");
  panelTestsWithRosRunning.navigationPanelShouldBeOpen(true);
  closePanelIfOpen("navigation");
  panelTestsWithRosRunning.navigationPanelShouldBeOpen(false);

  // Remote Control
  openPanelIfClosed("remote-control");
  panelTestsWithRosRunning.remoteControlPanelShouldBeOpen(true);
  closePanelIfOpen("remote-control");
  panelTestsWithRosRunning.remoteControlPanelShouldBeOpen(false);

  stopROS();

  checkPostRosShutdownStatus();
});

// TODO: In other files
// TODO: Test Unplugging
// TODO: Test Make Map
// TODO: Test Load Map
// TODO: Test Remote Control
// TODO: Test color follower?

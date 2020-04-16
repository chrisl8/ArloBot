import {
  resetRobotService,
  initialPageLoadItemsVisible,
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import {
  startROS,
  checkInitialTelemetry,
  stopROS,
  checkPostRosShutdownStatus
} from "../support/rosStartStop";

import panelTestsWithRosRunning, {
  additionalItemsAreVisibleWhenRosIsRunning
} from "../support/panelTestsWithRosRunning";

import panelTestsWithRosOff from "../support/panelTestsWithRosOff";

// TODO: WIP
describe("Start ROS and test Navigation Panel", () => {
  // TODO: You may want to comment out all of the start/stop code while testing this,
  //       So that you can just repeatedly test the actual map load bits instead of
  //       stopping/starting ROS and testing every bit of it repeatedly.
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

  // Navigation
  openPanelIfClosed("navigation");
  panelTestsWithRosRunning.navigationPanelShouldBeOpen(true);

  it("can load a map", () => {
    // TODO: Load Map
    cy.get("#load-map-button").should("be.visible");

    cy.get("#load-map-button").click();

    cy.contains("Pick a map to load:").should("be.visible");

    cy.get("#load-map-button-basement").should("be.visible");

    cy.get("#load-map-button-basement").click();

    cy.contains("Load Map Starting up").should("be.visible");

    cy.contains("Load Map Starting up", { timeout: 30000 }).should(
      "not.be.visible"
    );

    cy.get("#load-map-button").should("not.be.visible");

    cy.get("#make-map-button").should("not.be.visible");

    cy.contains("Map is Loaded.").should("be.visible");

    cy.contains("Map load complete.").should("be.visible");
  });

  // TODO: Go to location.

  // TODO: Save new location.

  closePanelIfOpen("navigation");
  panelTestsWithRosRunning.navigationPanelShouldBeOpen(false);

  stopROS();

  checkPostRosShutdownStatus();
});

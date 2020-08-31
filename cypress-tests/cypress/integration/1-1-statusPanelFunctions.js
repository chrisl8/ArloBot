import {
  resetRobotService,
  initialPageLoadItemsVisible,
  openPanelIfClosed,
  closePanelIfOpen,
} from "../support/reusableTestsAndSetupTasks";

import { statusPanelShouldBeOpen } from "../support/panelTestsWithRosOff";

describe("Status Panel Functions", () => {
  resetRobotService();
  initialPageLoadItemsVisible();

  statusPanelShouldBeOpen(true);
  closePanelIfOpen("status");
  statusPanelShouldBeOpen(false);
  openPanelIfClosed("status");
  statusPanelShouldBeOpen(true);

  it("Status Panel should contain correct data", () => {
    cy.get("#laptopBatteryStatusButton")
      .contains("span", "100%", { timeout: 10000 })
      .should("be.visible");
    cy.get("#laptopBatteryStatusButton").should("have.class", "btn-success");

    cy.get("#laptopChargedStatusButton")
      .contains("span", "Yes")
      .should("be.visible");
    cy.get("#laptopChargedStatusButton").should("have.class", "btn-success");

    cy.get("#pluggedInStatusButton")
      .contains("span", "Yes")
      .should("be.visible");
    cy.get("#pluggedInStatusButton").should("have.class", "btn-danger");

    cy.get("#mapStatusButton").contains("span", "None").should("be.visible");
    cy.get("#mapStatusButton").should("not.have.class", "btn-success");

    cy.get("#debuggingStatusButton")
      .contains("span", "No")
      .should("be.visible");
    cy.get("#debuggingStatusButton").should("not.have.class", "btn-warning");

    cy.get("#cameraStatusButton").contains("span", "Off").should("be.visible");
    cy.get("#cameraStatusButton").should("not.have.class", "btn-success");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");

    cy.get("#mycroftStatusButton").contains("span", "Up").should("be.visible");
    cy.get("#mycroftStatusButton").should("have.class", "btn-success");

    cy.get("#cloudStatusButton").contains("span", "Up").should("be.visible");
    cy.get("#cloudStatusButton").should("have.class", "btn-success");
  });

  it("debugging button should function", () => {
    cy.get("#statusScrollBox").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.get("#debuggingStatusButton")
      .contains("span", "No")
      .should("be.visible");
    cy.get("#debuggingStatusButton").should("not.have.class", "btn-warning");

    cy.get("#debuggingStatusButton").click();

    cy.get("#debuggingStatusButton")
      .contains("span", "Yes")
      .should("be.visible");
    cy.get("#debuggingStatusButton").should("have.class", "btn-warning");

    cy.contains("Handle Power without ROS").should("be.visible");
    cy.get("#startup-shutdown-card").contains("Start ROS").should("be.visible");
    cy.contains("Polling").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#debuggingStatusButton").click();

    cy.get("#debuggingStatusButton")
      .contains("span", "No")
      .should("be.visible");
    cy.get("#debuggingStatusButton").should("not.have.class", "btn-warning");

    cy.get("#statusScrollBox").should("not.be.visible");
  });

  it("camera status button should function", () => {
    // NOTE: This tests the camera button in the Status panel, not the Video tab
    cy.get("#cameraButton").contains("span", "Off").should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");
    cy.contains("Camera Off").should("be.visible");

    cy.get("#cameraStatusButton").contains("span", "Off").should("be.visible");

    cy.get("#cameraStatusButton").click();

    cy.get("#cameraButton").contains("span", "Off").should("be.visible");
    cy.get("#videoFeed").should("be.visible");

    cy.get("#cameraStatusButton").click();

    cy.get("#cameraButton").contains("span", "Off").should("be.visible");
    cy.get("#videoFeed").should("be.visible");

    cy.contains("Video - Camera Off").click();
    cy.get("#cameraButton").contains("span", "Off").should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");
  });

  it("master relay status button should function", () => {
    cy.get("#statusScrollBox").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");

    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay on.").should("be.visible");
    cy.contains("Master Relay on").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "On", { timeout: 15000 })
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");

    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay off.").should("be.visible");
    cy.contains("Master Relay off").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off", { timeout: 15000 })
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("not.be.visible");
  });

  // NOTE: Other buttons in the Status panel only function when things in the other panels
  // are activated. We will test them there.
});

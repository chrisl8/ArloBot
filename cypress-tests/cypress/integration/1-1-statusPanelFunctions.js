import {
  resetRobotService,
  correctItemsAreVisible
} from "../support/reusableTestsAndSetupTasks";

describe("Status Panel Functions", () => {
  resetRobotService();
  correctItemsAreVisible();

  it("status tab should close and open and contain correct data", () => {
    cy.contains("Status").click();
    cy.contains("Laptop Battery").should("not.be.visible");
    cy.contains("Laptop Fully Charged").should("not.be.visible");
    cy.contains("Plugged In").should("not.be.visible");
    cy.contains("Dangerous Doors Open").should("not.be.visible");
    cy.contains("Map").should("not.be.visible");
    cy.contains("Debugging").should("not.be.visible");
    cy.contains("Camera").should("not.be.visible");
    cy.contains("Master Relay").should("not.be.visible");

    cy.contains("Status").click();
    cy.contains("Laptop Battery").should("be.visible");
    cy.contains("Laptop Fully Charged").should("be.visible");
    cy.contains("Plugged In").should("be.visible");
    cy.contains("Dangerous Doors Open").should("be.visible");
    cy.contains("Map").should("be.visible");
    cy.contains("Debugging").should("be.visible");
    cy.contains("Camera").should("be.visible");
    cy.contains("Master Relay").should("be.visible");

    cy.get("#laptopBatteryStatusButton")
      .contains("span", "100%")
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

    cy.get("#doorsOpenStatusButton")
      .contains("span", "No")
      .should("be.visible");

    cy.get("#doorsOpenStatusButton").should("not.have.class", "btn-success");

    cy.get("#mapStatusButton")
      .contains("span", "None")
      .should("be.visible");

    cy.get("#mapStatusButton").should("not.have.class", "btn-success");

    cy.get("#debuggingStatusButton")
      .contains("span", "No")
      .should("be.visible");

    cy.get("#debuggingStatusButton").should("not.have.class", "btn-warning");

    cy.get("#cameraStatusButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#cameraStatusButton").should("not.have.class", "btn-success");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
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
    cy.contains("Start ROS").should("be.visible");
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
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");
    cy.contains("Camera Off").should("be.visible");

    cy.get("#cameraStatusButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#cameraStatusButton").click();

    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#videoFeed").should("be.visible");

    cy.get("#cameraStatusButton").click();

    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#videoFeed").should("be.visible");

    cy.contains("Video - Camera Off").click();
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("not.be.visible");
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
      .contains("span", "On")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");

    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay off.").should("be.visible");
    cy.contains("Master Relay off").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("not.be.visible");
  });

  // NOTE: Other buttons in the Status panel only function when things in the other panels
  // are activated. We will test them there.
});

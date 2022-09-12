import {
  resetRobotService,
  initialPageLoadItemsVisible,
  setSoundToQuiet,
  setIdleToTimeout,
  openPanelIfClosed,
  closePanelIfOpen,
} from "../support/reusableTestsAndSetupTasks";

import {
  videoPanelShouldBeOpen,
  startupShutdownPanelShouldBeOpen,
} from "../support/panelTestsWithRosOff";

describe("Behavior Panel Functions", () => {
  resetRobotService();
  setSoundToQuiet();
  setIdleToTimeout();
  initialPageLoadItemsVisible();

  openPanelIfClosed("relays");
  openPanelIfClosed("robot-service-log");
  closePanelIfOpen("startup-shutdown");
  startupShutdownPanelShouldBeOpen(false);
  // videoPanelShouldBeOpen(false);
  // openPanelIfClosed("video");
  // videoPanelShouldBeOpen(true);

  // Test current static image

  // it("turn on camera", () => {
  //   cy.contains("Video - Camera Off").should("be.visible");
  //
  //   cy.get("#fiveVoltRelayButton").contains("span", "Off").should("be.visible");
  //   cy.get("#fiveVoltRelayButton").should("not.have.class", "btn-success");
  //
  //   cy.get("#masterRelayStatusButton")
  //     .contains("span", "Off", { timeout: 15000 })
  //     .should("be.visible");
  //   cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  //
  //   cy.get("#cameraStatusButton").contains("span", "Off").should("be.visible");
  //   cy.get("#cameraStatusButton").should("not.have.class", "btn-success");
  //
  //   cy.get("#cameraButton").contains("span", "Off").should("be.visible");
  //   cy.get("#videoFeed").should("be.visible");
  //   cy.get("#cameraButton").should("not.have.class", "btn-success");
  //
  //   cy.get("#cameraButton").click();
  //
  //   cy.contains("Finding Camera C615", { timeout: 10000 }).should("be.visible");
  //   cy.contains("Starting Camera").should("be.visible");
  //
  //   cy.get("#cameraStatusButton").contains("span", "On").should("be.visible");
  //   cy.get("#cameraStatusButton").should("have.class", "btn-success");
  //
  //   cy.get("#cameraButton").contains("span", "On").should("be.visible");
  //   cy.get("#cameraButton").should("have.class", "btn-success");
  //   cy.get("#videoFeed").should("be.visible");
  //   cy.contains("Camera ON").should("be.visible");
  //
  //   cy.get("#videoFeed")
  //     .should("have.attr", "src")
  //     .should("include", "action=stream");
  //
  //   cy.wait(30000); // Visually confirm the camera feed is working.
  //   // cy.pause(); // If you want to manually verify and tell it to continue.
  // });
  //
  // it("turn off camera", () => {
  //   cy.get("#cameraButton").click();
  //
  //   cy.contains("Camera ended normally.").should("be.visible");
  //
  //   cy.get("#cameraStatusButton").contains("span", "Off").should("be.visible");
  //   cy.get("#cameraStatusButton").should("not.have.class", "btn-success");
  //
  //   cy.get("#cameraButton").contains("span", "Off").should("be.visible");
  //   cy.get("#cameraButton").should("not.have.class", "btn-success");
  //   cy.get("#videoFeed").should("be.visible");
  //   cy.get("#videoFeed")
  //     .should("have.attr", "src")
  //     .should("include", "xscreen.png");
  //
  //   cy.get("#masterRelayStatusButton")
  //     .contains("span", "Off", { timeout: 15000 })
  //     .should("be.visible");
  //   cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  // });
});

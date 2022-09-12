import {
  resetRobotService,
  openPanelIfClosed,
  closePanelIfOpen,
} from "../support/reusableTestsAndSetupTasks";

import { startupShutdownPanelShouldBeOpen } from "../support/panelTestsWithRosOff";

describe("Toggle Light One", () => {
  resetRobotService();

  it("page loads correctly", () => {
    // cy.contains("Master Relay").should("be.visible");

    // cy.get("#masterRelayStatusButton")
    //   .contains("span", "Off", { timeout: 15000 })
    //   .should("be.visible");

    cy.contains("Light One").should("not.be.visible");

    cy.contains("Relays").should("be.visible");
  });

  closePanelIfOpen("startup-shutdown");
  startupShutdownPanelShouldBeOpen(false);
  openPanelIfClosed("robot-service-log");

  // it("toggle Master Relay On", () => {
  //   cy.get("#masterRelayStatusButton").click();
  //
  //   cy.contains("Switching Master Relay on.").should("be.visible");
  //
  //   cy.contains("Master Relay on").should("be.visible");
  //
  //   cy.get("#masterRelayStatusButton")
  //     .contains("span", "On", { timeout: 15000 })
  //     .should("be.visible");
  //   cy.get("#masterRelayStatusButton").should("have.class", "btn-success");
  // });

  openPanelIfClosed("relays");

  it("relays panel is open", () => {
    cy.contains("Light One").should("be.visible");

    cy.get("#lightOneRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });

  it("can toggle light one on", () => {
    cy.contains("Light One").click();

    cy.contains("Relay 8 on").should("be.visible");

    cy.get("#lightOneRelayButton").contains("span", "On").should("be.visible");
    cy.get("#lightOneRelayButton").should("have.class", "btn-success");
  });

  it("pause to check that light one is on", () => {
    cy.wait(1000); // Use this to give time to see it working, but move on without input
    // cy.pause(); // Use this if you want to visually verify without a time limit.
  });

  it("can toggle light one off", () => {
    cy.contains("Light One").click();

    cy.contains("Relay 8 off").should("be.visible");

    cy.get("#lightOneRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });

  it("can toggle light two on", () => {
    cy.contains("Light Two").click();

    cy.contains("Relay 4 on").should("be.visible");

    cy.get("#lightTwoRelayButton").contains("span", "On").should("be.visible");
    cy.get("#lightTwoRelayButton").should("have.class", "btn-success");
  });

  it("pause to check that light two is on", () => {
    cy.wait(1000); // Use this to give time to see it working, but move on without input
    // cy.pause(); // Use this if you want to visually verify without a time limit.
  });

  it("can toggle light two off", () => {
    cy.contains("Light Two").click();

    cy.contains("Relay 4 off").should("be.visible");

    cy.get("#lightTwoRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");
  });

  closePanelIfOpen("relays");

  // it("toggle Master Relay Off", () => {
  //   cy.get("#masterRelayStatusButton").click();
  //
  //   cy.contains("Switching Master Relay off.").should("be.visible");
  //
  //   cy.contains("Master Relay off").should("be.visible");
  //
  //   cy.get("#masterRelayStatusButton")
  //     .contains("span", "Off", { timeout: 15000 })
  //     .should("be.visible");
  //   cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  // });
});

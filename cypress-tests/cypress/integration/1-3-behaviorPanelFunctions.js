import {
  resetRobotService,
  initialPageLoadItemsVisible,
  setSoundToQuiet,
  setIdleToTimeout,
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import {
  behaviorPanelShouldBeOpen,
  startupShutdownPanelShouldBeOpen
} from "../support/panelTestsWithRosOff";

describe("Behavior Panel Functions", () => {
  resetRobotService();
  setSoundToQuiet();
  setIdleToTimeout();
  initialPageLoadItemsVisible();

  behaviorPanelShouldBeOpen(false);

  openPanelIfClosed("behavior");

  behaviorPanelShouldBeOpen(true);

  openPanelIfClosed("relays");
  openPanelIfClosed("robot-service-log");
  closePanelIfOpen("startup-shutdown");
  startupShutdownPanelShouldBeOpen(false);

  it("should respond in text box when asked to speak when in quiet mode", () => {
    cy.get("#say-something").should("be.visible");

    cy.get("#say-something").type(
      "Forget. Erase memory banks concerning tennis. [whirring sound] Memory erased."
    );

    cy.contains("Speak").click();

    cy.get("#robot-said-text")
      .contains("I cannot reply, because I was asked to be quiet.")
      .should("be.visible");
  });

  it("should respond in text box when asked a question in quiet mode", () => {
    cy.get("#ask-something").should("be.visible");

    cy.get("#ask-something").type("What is your name?");

    cy.contains("Ask").click();

    cy.get("#robot-said-text")
      .contains("I cannot reply, because I was asked to be quiet.")
      .should("be.visible");
  });

  it("Idle button should work", () => {
    cy.get("#idle-timeout-button")
      .contains("Timeout")
      .should("have.class", "brightly-negative-text");

    cy.get("#idle-timeout-button").click();

    cy.contains("Idle Timer Stopped.").should("be.visible");

    cy.get("#idle-timeout-button")
      .contains("Never")
      .should("have.class", "brightly-positive-text");

    cy.get("#idle-timeout-button").click();

    cy.contains("Idle Timer Restarted.").should("be.visible");

    cy.get("#idle-timeout-button")
      .contains("Timeout")
      .should("have.class", "brightly-negative-text");
  });

  it("Sound button should toggle to Talk", () => {
    cy.get("#talk-bequiet-button")
      .contains("Quiet")
      .should("have.class", "brightly-negative-text");

    cy.get("#talk-bequiet-button").click();

    cy.get("#talk-bequiet-button")
      .contains("Talk")
      .should("have.class", "brightly-positive-text");
  });

  it("should speak audibly when asked to speak when in Talk mode", () => {
    cy.get("#say-something").should("be.visible");

    cy.get("#say-something").clear();

    cy.get("#say-something").type(
      "Forget. Erase memory banks concerning tennis. Memory erased."
    );

    cy.contains("Speak").click();

    cy.get("#robot-said-text")
      .contains("Forget. Erase memory banks concerning tennis. Memory erased.")
      .should("be.visible");

    cy.wait(1000); // Talking can be slow.
  });

  it("should respond verbally box when asked a question in Talk mode", () => {
    cy.get("#ask-something").should("be.visible");

    cy.get("#ask-something").clear();

    cy.get("#ask-something").type("What is your name?");

    cy.contains("Ask").click();

    /*
     * cat /opt/mycroft/skills/mycroft-configuration.mycroftai/dialog/en-us/my.name.is.dialog
     * my device name is {{name}}
     * I'm named {{name}}
     */

    cy.get("#robot-said-text")
      .contains("name")
      .should("be.visible");

    cy.get("#robot-said-text")
      .contains("TwoFlower")
      .should("be.visible");
  });

  it("Sound button should toggle to Quiet", () => {
    cy.get("#talk-bequiet-button")
      .contains("Talk")
      .should("have.class", "brightly-positive-text");

    cy.get("#talk-bequiet-button").click();

    cy.get("#talk-bequiet-button")
      .contains("Quiet")
      .should("have.class", "brightly-negative-text");
  });

  it("Blinky Lights button should turn on", () => {
    cy.get("#blinky-lights-button").should("not.have.class", "btn-warning");

    cy.contains("Blinky Lights").click();

    cy.contains("Switching Master Relay on.").should("be.visible");
    cy.contains("Master Relay on").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "On", { timeout: 15000 })
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");

    cy.contains("Relay 3 on").should("be.visible");

    cy.get("#arduinoRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#arduinoRelayButton").should("have.class", "btn-success");

    cy.contains("Relay 7 on").should("be.visible");

    cy.get("#fiveVoltRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#fiveVoltRelayButton").should("have.class", "btn-success");

    cy.get("#blinky-lights-button").should("have.class", "btn-warning");

    cy.wait(30000); // Give the magic time to happen.
    // cy.pause(); // If you want to manually verify and tell it to continue.
  });

  it("Blinky Lights button should turn off", () => {
    // NOTE: You can ONLY truly verify this visually
    cy.get("#blinky-lights-button").should("have.class", "btn-warning");

    cy.contains("Blinky Lights").click();

    cy.wait(15000); // Watch to ensure it worked.
    // cy.pause(); // If you want to manually verify and tell it to continue.

    // TODO: The lights do NOT always go off.

    cy.get("#blinky-lights-button").should("not.have.class", "btn-warning");
  });

  it("Should be possible to manually clean up", () => {
    cy.get("#arduinoRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#arduinoRelayButton").should("have.class", "btn-success");

    cy.get("#arduinoRelayButton").click();

    cy.contains("Relay 3 off").should("be.visible");

    cy.get("#arduinoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#arduinoRelayButton").should("not.have.class", "btn-success");

    cy.get("#fiveVoltRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#fiveVoltRelayButton").should("have.class", "btn-success");

    cy.get("#fiveVoltRelayButton").click();

    cy.contains("Relay 7 off").should("be.visible");

    cy.get("#fiveVoltRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#fiveVoltRelayButton").should("not.have.class", "btn-success");

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
  });
});

// TODO: NOTE: Some features of this tab do not exist unless the robot is running. Those will be tested elsewhere.

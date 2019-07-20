function resetRobotService() {
  it("reset robot service for a fresh start", () => {
    cy.visit("");

    cy.contains("Robot is Offline!").should("be.visible");

    cy.contains("Reset Robot Server").click();

    cy.contains("Robot is Offline!").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("Behavior").click();
    cy.contains("Hello my name is two flower").should("be.visible");

    cy.contains("Behavior").click();
  });
}

function correctItemsAreVisible() {
  it("correct items are visible on the screen", () => {
    cy.contains("Starting behaviors.").should("be.visible");
    cy.contains("Waiting for StartROS request.").should("be.visible");
    cy.contains("Emergency STOP").should("be.visible");
    cy.contains("Status").should("be.visible");
    cy.contains("Laptop Battery").should("be.visible");
    cy.contains("Laptop Fully Charged").should("be.visible");
    cy.contains("Plugged In").should("be.visible");
    cy.contains("Dangerous Doors Open").should("be.visible");
    cy.contains("Map").should("be.visible");
    cy.contains("Debugging").should("be.visible");
    cy.contains("Camera").should("be.visible");
    cy.contains("Master Relay").should("be.visible");
    cy.contains("Relays").should("be.visible");
    cy.contains("Behavior").should("be.visible");
    cy.contains("Startup/Shutdown").should("be.visible");
    cy.contains("ROS Stopped").should("be.visible");
    cy.contains("Start ROS").should("be.visible");
    cy.contains("Reset Robot Server").should("be.visible");
    cy.contains("Unplug").should("be.visible");
    cy.contains("Robot Service Log").should("be.visible");
    cy.contains("Video - Camera Off").should("be.visible");
    cy.contains("Camera Off").should("be.visible");
    cy.contains("https://github.com/chrisl8/ArloBot").should("be.visible");
    cy.get("#settings").should("be.visible");
  });
}

function setSoundToQuiet() {
  it("should set sound to Quiet if currently set to Talk", () => {
    cy.contains("Behavior").click();
    cy.get("#talk-bequiet-button")
      .contains("Talk")
      .each($elm => {
        cy.wrap($elm).then(() => {
          if ($elm.hasClass("brightly-positive-text")) {
            cy.get("#talk-bequiet-button").click();
          }
        });
      });

    cy.get("#talk-bequiet-button")
      .contains("Quiet")
      .should("have.class", "brightly-negative-text");

    cy.contains("Behavior").click();
  });
}

function setIdleToTimeout() {
  it("should set Idle to Timeout, not Never", () => {
    cy.contains("Behavior").click();
    cy.get("#idle-timeout-button")
      .contains("Never")
      .each($elm => {
        cy.wrap($elm).then(() => {
          if ($elm.hasClass("brightly-positive-text")) {
            cy.get("#idle-timeout-button").click();
          }
        });
      });

    cy.get("#idle-timeout-button")
      .contains("Timeout")
      .should("have.class", "brightly-negative-text");

    cy.contains("Behavior").click();
  });
}

function openRelayPanel() {
  it("should open Relay Panel", () => {
    cy.contains("Relays").click();

    cy.contains("Empty").should("be.visible");
    cy.contains("Right Motor").should("be.visible");
    cy.contains("Arduino").should("be.visible");
    cy.contains("Light Two").should("be.visible");
    cy.contains("Left Motor").should("be.visible");
    cy.contains("Five Volt").should("be.visible");
    cy.contains("Light One").should("be.visible");

    cy.get("#emptyRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton").should("not.have.class", "btn-success");

    cy.get("#rightMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#rightMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#arduinoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#arduinoRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightTwoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");

    cy.get("#emptyRelayButton5")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton5").should("not.have.class", "btn-success");

    cy.get("#leftMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#leftMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#fiveVoltRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#fiveVoltRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });
}

function openServiceLogPanel() {
  it("should open Robot Service Log Panel", () => {
    cy.get("#statusScrollBox").should("not.be.visible");
    cy.contains("ROSLIB Websocket closed").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");
    cy.contains("ROSLIB Websocket closed").should("be.visible");
  });
}

function closeStartupShutdownPanel() {
  it("should close Startup/Shutdown Panel", () => {
    cy.contains("Start ROS").should("be.visible");
    cy.contains("Reset Robot Server").should("be.visible");
    cy.contains("Unplug").should("be.visible");

    cy.contains("Startup/Shutdown").click();

    cy.contains("Start ROS").should("not.be.visible");
    cy.contains("Reset Robot Server").should("not.be.visible");
    cy.contains("Unplug").should("not.be.visible");
  });
}

function openVideoPanel() {
  it("should open Video Panel", () => {
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");

    cy.contains("Video - Camera Off").click();
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#cameraButton").should("not.have.class", "btn-success");
    cy.get("#videoFeed").should("be.visible");
    cy.get("#videoFeed")
      .should("have.attr", "src")
      .should("include", "xscreen.png");
  });
}

module.exports = {
  resetRobotService,
  correctItemsAreVisible,
  setSoundToQuiet,
  setIdleToTimeout,
  openRelayPanel,
  openServiceLogPanel,
  closeStartupShutdownPanel,
  openVideoPanel
};

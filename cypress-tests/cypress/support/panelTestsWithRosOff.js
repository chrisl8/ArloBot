function relayPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Relay Panel should be ${openClosed}`, () => {
    cy.contains("Relays").should("be.visible");
    cy.contains("Empty").should(visibilitystatus);
    cy.contains("Right Motor").should(visibilitystatus);
    cy.contains("Arduino").should(visibilitystatus);
    cy.contains("Light Two").should(visibilitystatus);
    cy.contains("Left Motor").should(visibilitystatus);
    cy.contains("Five Volt").should(visibilitystatus);
    cy.contains("Light One").should(visibilitystatus);
  });
}

function relayPanelCorrectInitialState() {
  it("Relay Panel entries should have correct initial settings", () => {
    cy.get("#emptyRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#emptyRelayButton").should("not.have.class", "btn-success");

    cy.get("#rightMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#rightMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#arduinoRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#arduinoRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightTwoRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");

    cy.get("#emptyRelayButton5").contains("span", "Off").should("be.visible");
    cy.get("#emptyRelayButton5").should("not.have.class", "btn-success");

    cy.get("#leftMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#leftMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#fiveVoltRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#fiveVoltRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightOneRelayButton").contains("span", "Off").should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });
}

function statusPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Status Panel should be ${openClosed}`, () => {
    cy.contains("Laptop Battery").should(visibilitystatus);
    cy.contains("Laptop Fully Charged").should(visibilitystatus);
    cy.contains("Plugged In").should(visibilitystatus);
    cy.contains("Map").should(visibilitystatus);
    cy.contains("Debugging").should(visibilitystatus);
    cy.contains("Camera").should(visibilitystatus);
    cy.contains("Master Relay").should(visibilitystatus);
    cy.contains("Mycroft").should(visibilitystatus);
    cy.contains("Cloud").should(visibilitystatus);
  });
}

function behaviorPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Behavior Panel should be ${openClosed}`, () => {
    cy.get("#say-something").should(visibilitystatus);
    cy.contains("Speak").should(visibilitystatus);
    cy.get("#ask-something").should(visibilitystatus);
    cy.contains("Ask").should(visibilitystatus);
    cy.contains("Response:").should(visibilitystatus);
    cy.contains("Hello, my name is ").should(visibilitystatus);
    cy.contains("Never").should(visibilitystatus);
    cy.contains("Idle").should(visibilitystatus);
    cy.contains("Timeout").should(visibilitystatus);
    cy.contains("Talk").should(visibilitystatus);
    cy.contains("Sound").should(visibilitystatus);
    cy.contains("Quiet").should(visibilitystatus);
    cy.contains("Blinky Lights").should(visibilitystatus);
  });
}

function videoPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Video Panel should be ${openClosed}`, () => {
    cy.get("#cameraButton").contains("span", "Off").should(visibilitystatus);
    cy.get("#videoFeed").should(visibilitystatus);
    cy.get("#videoFeed")
      .should("have.attr", "src")
      .should("include", "xscreen.png");
  });
}

function startupShutdownPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Startup/Shutdown Panel should be ${openClosed}`, () => {
    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .should(visibilitystatus);
    cy.get("#startup-shutdown-card")
      .contains("Reset Robot Server")
      .should(visibilitystatus);
    cy.get("#startup-shutdown-card")
      .contains("Unplug")
      .should(visibilitystatus);
  });
}

function robotServiceLogPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Robot Service Log Panel should be ${openClosed}`, () => {
    cy.get("#statusScrollBox").should(visibilitystatus);
    cy.contains("ROSLIB Websocket closed").should(visibilitystatus);
  });
}

module.exports = {
  relayPanelShouldBeOpen,
  relayPanelCorrectInitialState,
  statusPanelShouldBeOpen,
  behaviorPanelShouldBeOpen,
  videoPanelShouldBeOpen,
  startupShutdownPanelShouldBeOpen,
  robotServiceLogPanelShouldBeOpen,
};

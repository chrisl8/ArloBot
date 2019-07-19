describe("Relay Panel Functions", () => {
  it("reset robot service for a fresh start", () => {
    cy.visit("");

    cy.contains("Reset Robot Server").click();

    cy.contains("Robot is Offline!").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("Behavior").click();
    cy.contains("Hello my name is two flower").should("be.visible");

    cy.contains("Behavior").click();
  });

  it("correct items are visible on the screen", () => {
    cy.contains("Starting behaviors.").should("be.visible");
    cy.contains("Waiting for StartROS request.").should("be.visible");
    cy.contains("Emergency STOP").should("be.visible");
    cy.contains("Status").should("be.visible");
    cy.contains("Relays").should("be.visible");
    cy.contains("Behavior").should("be.visible");
    cy.contains("Startup/Shutdown").should("be.visible");
    cy.contains("ROS Stopped").should("be.visible");
    cy.contains("Robot Service Log").should("be.visible");
    cy.contains("Video").should("be.visible");
    cy.contains("Camera Off").should("be.visible");
    cy.contains("https://github.com/chrisl8/ArloBot").should("be.visible");
    cy.get("#settings").should("be.visible");
  });

  it("relay tab should be closed and open properly", () => {
    cy.contains("Empty").should("not.be.visible");
    cy.contains("Right Motor").should("not.be.visible");
    cy.contains("Arduino").should("not.be.visible");
    cy.contains("Light Two").should("not.be.visible");
    cy.contains("Left Motor").should("not.be.visible");
    cy.contains("Five Volt").should("not.be.visible");
    cy.contains("Light One").should("not.be.visible");

    cy.contains("Relays").should("be.visible");

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

  it("set up panels to monitor log", () => {
    cy.contains("Status").click();
    cy.contains("Laptop Battery").should("not.be.visible");
    cy.contains("Laptop Fully Charged").should("not.be.visible");
    cy.contains("Plugged In").should("not.be.visible");
    cy.contains("Dangerous Doors Open").should("not.be.visible");
    cy.contains("Map").should("not.be.visible");
    cy.contains("Debugging").should("not.be.visible");
    cy.contains("Camera").should("not.be.visible");
    cy.contains("Master Relay").should("not.be.visible");

    cy.contains("Startup/Shutdown").click();
    cy.contains("Start ROS").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");
  });

  it("Empty Relay button should function", () => {
    cy.get("#emptyRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton").should("not.have.class", "btn-success");

    cy.get("#emptyRelayButton").click();

    cy.contains("Relay 1 on").should("be.visible");

    cy.get("#emptyRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#emptyRelayButton").should("have.class", "btn-success");

    cy.get("#emptyRelayButton").click();

    cy.contains("Relay 1 off").should("be.visible");

    cy.get("#emptyRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton").should("not.have.class", "btn-success");
  });

  it("Right Motor relay button should function", () => {
    cy.get("#rightMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#rightMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#rightMotorRelayButton").click();

    cy.contains("Relay 2 on").should("be.visible");

    cy.get("#rightMotorRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#rightMotorRelayButton").should("have.class", "btn-success");

    cy.get("#rightMotorRelayButton").click();

    cy.contains("Relay 2 off").should("be.visible");

    cy.get("#rightMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#rightMotorRelayButton").should("not.have.class", "btn-success");
  });

  it("Arduino relay button should function", () => {
    cy.get("#arduinoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#arduinoRelayButton").should("not.have.class", "btn-success");

    cy.get("#arduinoRelayButton").click();

    cy.contains("Relay 3 on").should("be.visible");

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
  });

  it("Light Two relay button should function", () => {
    cy.get("#lightTwoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightTwoRelayButton").click();

    cy.contains("Relay 4 on").should("be.visible");

    cy.get("#lightTwoRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("have.class", "btn-success");

    cy.get("#lightTwoRelayButton").click();

    cy.contains("Relay 4 off").should("be.visible");

    cy.get("#lightTwoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");
  });

  it("Second Empty relay button should function", () => {
    cy.get("#emptyRelayButton5")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton5").should("not.have.class", "btn-success");

    cy.get("#emptyRelayButton5").click();

    cy.contains("Relay 5 on").should("be.visible");

    cy.get("#emptyRelayButton5")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#emptyRelayButton5").should("have.class", "btn-success");

    cy.get("#emptyRelayButton5").click();

    cy.contains("Relay 5 off").should("be.visible");

    cy.get("#emptyRelayButton5")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#emptyRelayButton5").should("not.have.class", "btn-success");
  });

  it("Left Motor relay button should function", () => {
    cy.get("#leftMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#leftMotorRelayButton").should("not.have.class", "btn-success");

    cy.get("#leftMotorRelayButton").click();

    cy.contains("Relay 6 on").should("be.visible");

    cy.get("#leftMotorRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#leftMotorRelayButton").should("have.class", "btn-success");

    cy.get("#leftMotorRelayButton").click();

    cy.contains("Relay 6 off").should("be.visible");

    cy.get("#leftMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#leftMotorRelayButton").should("not.have.class", "btn-success");
  });

  it("Five Volt relay button should function", () => {
    cy.get("#fiveVoltRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#fiveVoltRelayButton").should("not.have.class", "btn-success");

    cy.get("#fiveVoltRelayButton").click();

    cy.contains("Relay 7 on").should("be.visible");

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
  });

  it("Light One relay button should function", () => {
    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");

    cy.get("#lightOneRelayButton").click();

    cy.contains("Relay 8 on").should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("have.class", "btn-success");

    cy.get("#lightOneRelayButton").click();

    cy.contains("Relay 8 off").should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });
});

// TODO: Flipping these buttons doesn't actually test the function in this case for many reasons,
// TODO: for instance because the Master Relay is off.
// TODO: Actual Robot functions should be tested at some point.

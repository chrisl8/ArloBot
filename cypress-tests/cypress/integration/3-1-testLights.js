describe("Toggle Light One", () => {
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

  it("page loads correctly", () => {
    cy.contains("Master Relay").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.contains("Light One").should("not.be.visible");

    cy.contains("Relays").should("be.visible");
  });

  it("set up panels to monitor log", () => {
    cy.contains("Startup/Shutdown").click();
    cy.contains("Start ROS").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");
  });

  it("toggle Master Relay On", () => {
    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay on.").should("be.visible");

    cy.contains("Master Relay on").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");
  });

  it("can open relay panel", () => {
    cy.contains("Relays").click();

    cy.contains("Light One").should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });

  it("can toggle light one on", () => {
    cy.contains("Light One").click();

    cy.contains("Relay 8 on").should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("have.class", "btn-success");
  });

  it("pause to check that light one is on", () => {
    cy.wait(1000); // Use this to give time to see it working, but move on without input
    // cy.pause(); // Use this if you want to visually verify without a time limit.
  });

  it("can toggle light one off", () => {
    cy.contains("Light One").click();

    cy.contains("Relay 8 off").should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightOneRelayButton").should("not.have.class", "btn-success");
  });

  it("can toggle light two on", () => {
    cy.contains("Light Two").click();

    cy.contains("Relay 4 on").should("be.visible");

    cy.get("#lightTwoRelayButton")
      .contains("span", "On")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("have.class", "btn-success");
  });

  it("pause to check that light two is on", () => {
    cy.wait(1000); // Use this to give time to see it working, but move on without input
    // cy.pause(); // Use this if you want to visually verify without a time limit.
  });

  it("can toggle light two off", () => {
    cy.contains("Light Two").click();

    cy.contains("Relay 4 off").should("be.visible");

    cy.get("#lightTwoRelayButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#lightTwoRelayButton").should("not.have.class", "btn-success");
  });

  it("can close relay panel", () => {
    cy.contains("Relays").click();

    cy.contains("Light One").should("not.be.visible");

    cy.contains("Relays").should("be.visible");
  });

  it("toggle Master Relay Off", () => {
    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay off.").should("be.visible");

    cy.contains("Master Relay off").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  });
});

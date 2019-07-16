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

  it("toggle Master Relay On", () => {
    cy.get("#masterRelayStatusButton").click();

    cy.get("#masterRelayStatusButton")
      .contains("span", "On")
      .should("be.visible");
  });

  it("can open relay panel", () => {
    cy.contains("Relays").click();

    cy.contains("Light One").should("be.visible");

    cy.get("#lightOneButton")
      .contains("span", "Off")
      .should("be.visible");
  });

  it("can toggle light one on", () => {
    cy.contains("Light One").click();

    cy.get("#lightOneButton")
      .contains("span", "On")
      .should("be.visible");
  });

  it("can toggle light one off", () => {
    cy.contains("Light One").click();

    cy.get("#lightOneButton")
      .contains("span", "Off")
      .should("be.visible");
  });

  it("can close relay panel", () => {
    cy.contains("Relays").click();

    cy.contains("Light One").should("not.be.visible");

    cy.contains("Relays").should("be.visible");
  });

  it("toggle Master Relay On", () => {
    cy.get("#masterRelayStatusButton").click();

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
  });

  it("service log should reflect activity", () => {
    cy.contains("Robot Service Log").click();

    cy.contains("ROSLIB Websocket closed").should("be.visible");
    cy.contains("Master Relay off").should("be.visible");
    cy.contains("Switching Master Relay off.").should("be.visible");
    cy.contains("Relay 8 off").should("be.visible");
    cy.contains("Relay 8 on").should("be.visible");
    cy.contains("Master Relay on").should("be.visible");
    cy.contains("Switching Master Relay on.").should("be.visible");
    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.contains("Robot Service Log").click();
  });
});

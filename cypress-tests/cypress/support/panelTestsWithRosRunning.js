function additionalItemsAreVisibleWhenRosIsRunning() {
  it("correct items are visible when ROS is running", () => {
    cy.get("#telemetry-card")
      .contains("Telemetry")
      .should("be.visible");

    cy.get("#sensors-card")
      .contains("Sensors")
      .should("be.visible");

    cy.get("#navigation-card")
      .contains("Navigation")
      .should("be.visible");

    cy.get("#remote-control-card")
      .contains("Remote Control")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .should("not.have.class", "btn-success");

    cy.get("#robot-service-log-card")
      .get("#set-all-doors-closed-button")
      .should("be.visible");
  });
}

module.exports = { additionalItemsAreVisibleWhenRosIsRunning };

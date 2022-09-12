function additionalItemsAreVisibleWhenRosIsRunning() {
  it("correct items are visible when ROS is running", () => {
    cy.get("#telemetry-card").contains("Telemetry").should("be.visible");

    cy.get("#sensors-card").contains("Sensors").should("be.visible");

    cy.get("#navigation-card").contains("Navigation").should("be.visible");

    cy.get("#remote-control-card")
      .contains("Remote Control")
      .should("be.visible");
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

    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket connected.")
      .should(visibilitystatus);

    cy.get("#robot-service-log-card")
      .get("#log-streamer-button")
      .should(visibilitystatus);
  });
}

function navigationPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Navigation Log Panel should be ${openClosed}`, () => {
    cy.get("#navigation-card").contains("Make Map").should(visibilitystatus);

    cy.get("#navigation-card").contains("Load Map").should(visibilitystatus);

    cy.get("#navigation-card")
      .contains("To make a new map")
      .should(visibilitystatus);

    cy.get("#navigation-card")
      .contains("The only way to load a map after making it")
      .should(visibilitystatus);
  });
}

function remoteControlPanelShouldBeOpen(trueFalse) {
  let visibilitystatus = "not.be.visible";
  let openClosed = "Closed";
  if (trueFalse) {
    visibilitystatus = "be.visible";
    openClosed = "Open";
  }
  it(`Remote Control Panel should be ${openClosed}`, () => {
    // cy.get("#remote-control-card")
    //   .contains("Plugged In")
    //   .should(visibilitystatus);

    cy.get("#remote-control-card")
      .contains("Use finger or mouse to drive robot!")
      .should(visibilitystatus);

    cy.get("#remote-control-card").within(() => {
      cy.get("#virtual-joystick-container").should(visibilitystatus);
    });
  });
}

module.exports = {
  additionalItemsAreVisibleWhenRosIsRunning,
  robotServiceLogPanelShouldBeOpen,
  navigationPanelShouldBeOpen,
  remoteControlPanelShouldBeOpen,
};

function startROS() {
  it("Start ROS", () => {
    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .click();

    cy.get("#startup-shutdown-card")
      .contains("Start ROS")
      .should("not.be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Starting...")
      .should("be.visible");

    cy.get("#startup-shutdown-card")
      .contains("Please wait, ROS is starting . . .")
      .should("be.visible");

    // Log messages:
    cy.get("#statusScrollBox")
      .contains("ROS is starting up . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Start ROS: Process starting!")
      .should("be.visible");
    // cy.contains("Clearing ROS Logs . . .").should("be.visible");
    // Clearing ROS Logs only happens if there ARE ROS Logs, so not consistent.
    cy.get("#statusScrollBox")
      .contains("Turning on Arlo Power supply . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("relay on 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Giving it 1 second to come online . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Master Relay on")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Turning on Five Volt power converter . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Camera 0 . . .", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Xbox Controller . . .", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Activity Board . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Checking Quick Start Board . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Hardware Check SUCCESS! All devices found.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Waiting for roscore to start . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Use kill", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ros.sh to close.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Rosapi started", { timeout: 10000 })
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROS successfully started.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket connected.")
      .should("be.visible");
  });
}

function stopROS() {
  it("Stop ROS", () => {
    cy.get("#startup-shutdown-card")
      .contains("Stop ROS")
      .click();

    // Log messages:
    cy.get("#statusScrollBox")
      .contains("ROS is starting up . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Running kill_ros.sh . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Killing everything, please wait...")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown:")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Waiting for Robot to close . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket closed")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Waiting for Robot to close . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Waiting for roscore to close . . .")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROS exited with code: 143")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Start ROS: Process Closed.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Everything Killed.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Turning off all relays")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: Turning off Arlo Power.")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Shutdown: relay off 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("kill_ros.sh closed with code 0")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("Master Relay off")
      .should("be.visible");
    cy.get("#statusScrollBox")
      .contains("ROSLIB Websocket closed")
      .should("be.visible");

    cy.get("#navigation-card").should("not.be.visible");
    cy.get("#remote-control-card").should("not.be.visible");
    cy.get("#telemetry-card").should("not.be.visible");
    cy.get("#sensors-card").should("not.be.visible");

    cy.contains("ROS process has closed.").should("be.visible");
    cy.contains("Waiting for StartROS request.").should("be.visible");
  });
}

module.exports = { startROS, stopROS };

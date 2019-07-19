describe("Behavior Panel Functions", () => {
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

  it("should set sound to Idle Timeout, not Never", () => {
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

  it("bahavior tab should be closed and open and contain correct data", () => {
    cy.get("#say-something").should("not.be.visible");
    cy.contains("Speak").should("not.be.visible");
    cy.get("#ask-something").should("not.be.visible");
    cy.contains("Ask").should("not.be.visible");
    cy.contains("Response:").should("not.be.visible");
    cy.contains("Hello my name is two flower").should("not.be.visible");
    cy.contains("Never").should("not.be.visible");
    cy.contains("Idle").should("not.be.visible");
    cy.contains("Timeout").should("not.be.visible");
    cy.contains("Talk").should("not.be.visible");
    cy.contains("Sound").should("not.be.visible");
    cy.contains("Quiet").should("not.be.visible");
    cy.contains("Blinky Lights").should("not.be.visible");

    cy.contains("Behavior").click();
    cy.get("#say-something").should("be.visible");
    cy.contains("Speak").should("be.visible");
    cy.get("#ask-something").should("be.visible");
    cy.contains("Ask").should("be.visible");
    cy.contains("Response:").should("be.visible");
    cy.contains("Hello my name is two flower").should("be.visible");
    cy.contains("Never").should("be.visible");
    cy.contains("Idle").should("be.visible");
    cy.contains("Timeout").should("be.visible");
    cy.contains("Talk").should("be.visible");
    cy.contains("Sound").should("be.visible");
    cy.contains("Quiet").should("be.visible");
    cy.contains("Blinky Lights").should("be.visible");
  });

  it("set up panels to monitor relays and log", () => {
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

    cy.contains("Startup/Shutdown").click();
    cy.contains("Start ROS").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");
  });

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
      .contains("span", "On")
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
      .contains("span", "On")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("have.class", "btn-success");

    cy.get("#masterRelayStatusButton").click();

    cy.contains("Switching Master Relay off.").should("be.visible");
    cy.contains("Master Relay off").should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#masterRelayStatusButton").should("not.have.class", "btn-success");
  });
});

// TODO: NOTE: Some features of this tab do not exist unless the robot is running. Those will be tested elsewhere.

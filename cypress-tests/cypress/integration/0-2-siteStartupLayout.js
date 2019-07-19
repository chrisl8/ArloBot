describe("site initial layout and page function", () => {
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
    cy.contains("Video").should("be.visible");
    cy.contains("Camera Off").should("be.visible");
    cy.contains("https://github.com/chrisl8/ArloBot").should("be.visible");
    cy.get("#settings").should("be.visible");
  });

  it("emergency stop button should cycle", () => {
    cy.contains("Emergency STOP").should("be.visible");
    cy.contains("Resume").should("not.be.visible");
    cy.contains("Emergency STOP").click();
    cy.contains("Emergency STOP").should("not.be.visible");
    cy.contains("Resume").should("be.visible");
    cy.contains("Resume").click();
    cy.contains("Emergency STOP").should("be.visible");
    cy.contains("Resume").should("not.be.visible");
  });

  it("status tab should close and open and contain correct data", () => {
    cy.contains("Status").click();
    cy.contains("Laptop Battery").should("not.be.visible");
    cy.contains("Laptop Fully Charged").should("not.be.visible");
    cy.contains("Plugged In").should("not.be.visible");
    cy.contains("Dangerous Doors Open").should("not.be.visible");
    cy.contains("Map").should("not.be.visible");
    cy.contains("Debugging").should("not.be.visible");
    cy.contains("Camera").should("not.be.visible");
    cy.contains("Master Relay").should("not.be.visible");

    cy.contains("Status").click();
    cy.contains("Laptop Battery").should("be.visible");
    cy.contains("Laptop Fully Charged").should("be.visible");
    cy.contains("Plugged In").should("be.visible");
    cy.contains("Dangerous Doors Open").should("be.visible");
    cy.contains("Map").should("be.visible");
    cy.contains("Debugging").should("be.visible");
    cy.contains("Camera").should("be.visible");
    cy.contains("Master Relay").should("be.visible");

    cy.get("#laptopBatteryStatusButton")
      .contains("span", "100%")
      .should("be.visible");

    cy.get("#laptopChargedStatusButton")
      .contains("span", "Yes")
      .should("be.visible");

    cy.get("#pluggedInStatusButton")
      .contains("span", "Yes")
      .should("be.visible");

    cy.get("#doorsOpenStatusButton")
      .contains("span", "No")
      .should("be.visible");

    cy.get("#mapStatusButton")
      .contains("span", "None")
      .should("be.visible");

    cy.get("#debuggingStatusButton")
      .contains("span", "No")
      .should("be.visible");

    cy.get("#cameraStatusButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#masterRelayStatusButton")
      .contains("span", "Off")
      .should("be.visible");
  });

  it("relay tab should open and close", () => {
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

    cy.get("#rightMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#arduinoRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#lightTwoRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#emptyRelayButton5")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#leftMotorRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#fiveVoltRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.get("#lightOneRelayButton")
      .contains("span", "Off")
      .should("be.visible");

    cy.contains("Relays").click();

    cy.contains("Empty").should("not.be.visible");
    cy.contains("Right Motor").should("not.be.visible");
    cy.contains("Arduino").should("not.be.visible");
    cy.contains("Light Two").should("not.be.visible");
    cy.contains("Left Motor").should("not.be.visible");
    cy.contains("Five Volt").should("not.be.visible");
    cy.contains("Light One").should("not.be.visible");
  });

  it("behavior tab should open and close", () => {
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

    cy.contains("Behavior").click();
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
  });

  it("startup/shutdown tab should open and close", () => {
    cy.contains("Start ROS").should("be.visible");
    cy.contains("Reset Robot Server").should("be.visible");
    cy.contains("Unplug").should("be.visible");

    cy.contains("Startup/Shutdown").click();
    cy.contains("Start ROS").should("not.be.visible");
    cy.contains("Reset Robot Server").should("not.be.visible");
    cy.contains("Unplug").should("not.be.visible");

    cy.contains("Startup/Shutdown").click();
    cy.contains("Start ROS").should("be.visible");
    cy.contains("Reset Robot Server").should("be.visible");
    cy.contains("Unplug").should("be.visible");
  });

  it("robot service log tab should open and close", () => {
    cy.get("#statusScrollBox").should("not.be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.get("#statusScrollBox").should("not.be.visible");
  });

  it("video tab should open and close", () => {
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");

    cy.contains("Video").click();
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("be.visible");
    cy.get("#videoFeed").should("be.visible");

    cy.contains("Video").click();
    cy.get("#cameraButton")
      .contains("span", "Off")
      .should("not.be.visible");
    cy.get("#videoFeed").should("not.be.visible");
  });

  it("settings page should work and close", () => {
    cy.get("#settings").should("be.visible");
    cy.get("#control").should("not.exist");

    cy.get("#settings").click();

    cy.get("#control").should("be.visible");
    cy.get("#settings").should("not.exist");

    cy.contains("robotName:").should("be.visible");
    cy.contains("idleTimeoutInMinutes: 15").should("be.visible");
    cy.contains("useMyCroft:").should("be.visible");
    cy.contains("rosLibDelay:").should("be.visible");
    cy.contains("useMasterPowerRelay:").should("be.visible");
    cy.contains("masterPowerRelayUniqueString:").should("be.visible");
    cy.contains("masterPowerRelayStringLocation:").should("be.visible");
    cy.contains("useLCD:").should("be.visible");
    cy.contains("LCDString:").should("be.visible");
    cy.contains("LCDLocation:").should("be.visible");
    cy.contains("useArduinoForBlinkenLights:").should("be.visible");
    cy.contains("arduinoUniqueString:").should("be.visible");
    cy.contains("arduinoStringLocation:").should("be.visible");
    cy.contains("useUSBrelay:").should("be.visible");
    cy.contains("use_xv11:").should("be.visible");
    cy.contains("camera0:").should("be.visible");
    cy.contains("camera0name:").should("be.visible");
    cy.contains("camera1:").should("be.visible");
    cy.contains("camera1name:").should("be.visible");
    cy.contains("wait_for_door_confirmation:").should("be.visible");
    cy.contains("hasXboxController:").should("be.visible");
    cy.contains("hasScanseSweep:").should("be.visible");
    cy.contains("arlobotModel:").should("be.visible");
    cy.contains("hasActivityBoard:").should("be.visible");
    cy.contains("hasQuickStartBoard:").should("be.visible");
    cy.contains("speechVolumeLevelDefault:").should("be.visible");
    cy.contains("defaultMicrophoneName:").should("be.visible");
    cy.contains("speechProgram:").should("be.visible");
    cy.contains("speech_engine:").should("be.visible");
    cy.contains("use_external_speaker:").should("be.visible");
    cy.contains("script_location:").should("be.visible");
    cy.contains("web_folder:").should("be.visible");
    cy.contains("webServerPort:").should("be.visible");
    cy.contains("launchBrowser:").should("be.visible");
    cy.contains("useQRcodes:").should("be.visible");
    cy.contains("qrCameraName:").should("be.visible");
    cy.contains("batteryConsideredFullAt:").should("be.visible");
    cy.contains("maxPingRangeAccepted:").should("be.visible");
    cy.contains("hasBeenEdited:").should("be.visible");

    cy.get("#control").click();

    cy.get("#settings").should("be.visible");
    cy.get("#control").should("not.exist");
  });
});

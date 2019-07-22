import {
  resetRobotService,
  initialPageLoadItemsVisible,
  setSoundToQuiet,
  setIdleToTimeout,
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import {
  relayPanelShouldBeOpen,
  relayPanelCorrectIntialState,
  statusPanelShouldBeOpen,
  behaviorPanelShouldBeOpen,
  startupShutdownPanelShouldBeOpen
} from "../support/panelTestsWithRosOff";

describe("site initial layout and page function", () => {
  resetRobotService();
  setSoundToQuiet();
  setIdleToTimeout();
  initialPageLoadItemsVisible();

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

  closePanelIfOpen("status");

  statusPanelShouldBeOpen(false);

  openPanelIfClosed("status");

  statusPanelShouldBeOpen(true);
  it("Status Panel entries should be set correctly", () => {
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

  relayPanelShouldBeOpen(false);

  openPanelIfClosed("relays");

  relayPanelShouldBeOpen(true);
  relayPanelCorrectIntialState();
  closePanelIfOpen("relays");

  relayPanelShouldBeOpen(false);

  behaviorPanelShouldBeOpen(false);
  openPanelIfClosed("behavior");
  behaviorPanelShouldBeOpen(true);
  closePanelIfOpen("behavior");
  behaviorPanelShouldBeOpen(false);

  openPanelIfClosed("startup-shutdown");
  startupShutdownPanelShouldBeOpen(true);

  openPanelIfClosed("robot-service-log");

  it("robot service log tab should be open", () => {
    cy.get("#statusScrollBox").should("be.visible");

    cy.contains("ROSLIB Websocket closed").should("be.visible");
  });

  closePanelIfOpen("robot-service-log");
  it("robot service log tab should be closed", () => {
    cy.get("#statusScrollBox").should("not.be.visible");
  });

  closePanelIfOpen("video");

  it("video tab should be closed", () => {
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

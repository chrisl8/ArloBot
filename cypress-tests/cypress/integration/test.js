import {
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import { relayPanelShouldBeOpen } from "../support/panelTestsWithRosOff";

import panelTestsWithRosRunning from "../support/panelTestsWithRosRunning";

describe("test", () => {
  it("successfully loads", () => {
    cy.visit("");
  });
});

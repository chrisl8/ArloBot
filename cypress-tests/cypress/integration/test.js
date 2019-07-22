import {
  openPanelIfClosed,
  closePanelIfOpen
} from "../support/reusableTestsAndSetupTasks";

import { relayPanelShouldBeOpen } from "../support/panelTestsWithRosOff";

describe("test", () => {
  it("successfully loads", () => {
    cy.visit("");
  });
  relayPanelShouldBeOpen(false);
  openPanelIfClosed("relays");
  relayPanelShouldBeOpen(true);
  closePanelIfOpen("relays");
  relayPanelShouldBeOpen(false);
});

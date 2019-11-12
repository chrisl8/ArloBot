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

  // Navigation
  openPanelIfClosed("navigation");
  panelTestsWithRosRunning.navigationPanelShouldBeOpen(true);

  it("can load a map", () => {
    // TODO: Load Map
    cy.get("#load-map-button").should("be.visible");

    cy.get("#load-map-button").click();

    cy.contains("Pick a map to load:").should("be.visible");

    cy.get("#load-map-button-basement").should("be.visible");

    cy.get("#load-map-button-basement").click();

    cy.contains("Load Map Starting up").should("be.visible");

    cy.contains("Load Map Starting up", { timeout: 30000 }).should(
      "not.be.visible"
    );

    cy.get("#load-map-button").should("not.be.visible");

    cy.get("#make-map-button").should("not.be.visible");

    cy.contains("Map is Loaded.").should("be.visible");

    cy.contains("Map load complete.").should("be.visible");
  });

  // TODO: Go to location.

  // TODO: Save new location.
});

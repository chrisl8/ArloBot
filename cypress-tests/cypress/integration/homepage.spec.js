/*
 * This is really just a test to make sure Cypress is working,
 * and the web server is running.
 *
 * I usually slap this in as my first test after installing Cypress.
 */

describe("The Home Page", () => {
  it("successfully loads", () => {
    cy.visit("/");
  });

  it("new project page loads correctly", () => {
    cy.contains("Robot is Offline!").should("be.visible");

    cy.contains("Starting behaviors.").should("be.visible");

    cy.contains("Waiting for StartROS request.").should("be.visible");
  });
});

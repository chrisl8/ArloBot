function openPanelIfClosed(id) {
  it(`open ${id} if it is closed`, () => {
    cy.get(`#${id}-card-body`).each(($elm) => {
      cy.wrap($elm).then(() => {
        if (!$elm.hasClass("show")) {
          cy.get($elm).parent().click();
          // Do not continue until animation is done.
          cy.get(`#${id}-card-body`).should("not.have.class", "collapsing");
        }
      });
    });
  });
}

function closePanelIfOpen(id) {
  it(`close ${id} if it is open`, () => {
    // Cannot close while it is still in motion, so wait for it to be done
    // if it was in motion.
    cy.get(`#${id}-card-body`).should("not.have.class", "collapsing");
    cy.get(`#${id}-card-body`).each(($elm) => {
      cy.wrap($elm).then(() => {
        console.log($elm);
        if ($elm.hasClass("show")) {
          cy.get(`#${id}-card`).within(() => {
            cy.get("div[class='card-header']").click();
            // Do not continue until animation is done.
            cy.get(`#${id}-card-body`).should("not.have.class", "collapsing");
          });
        }
      });
    });
  });
}

function resetRobotService() {
  setSoundToTalk();
  it("reset robot service for a fresh start", () => {
    cy.visit("");

    // cy.contains("Robot is Offline!").should("be.visible"); // TODO: Put this back
    //
    // cy.contains("Reset Robot Server").click(); // TODO: Put this back.

    cy.contains("Robot is Offline!", { timeout: 10000 }).should("be.visible");

    cy.contains("Robot Service Log", { timeout: 10000 }).click();

    cy.contains("ROSLIB Websocket closed").should("be.visible");

    cy.contains("Robot Service Log").click();

    cy.contains("Behavior").click();
    // cy.contains("Hello, my name is ").should("be.visible");

    cy.contains("Behavior").click();
  });
}

function initialPageLoadItemsVisible() {
  it("correct items are visible on the screen", () => {
    // cy.contains("Starting behaviors.").should("be.visible");
    cy.contains("Waiting for StartROS request.").should("be.visible");
    cy.contains("Emergency STOP").should("be.visible");
    cy.contains("Status").should("be.visible");
    // cy.contains("Laptop Battery").should("be.visible");
    // cy.contains("Laptop Fully Charged").should("be.visible");
    // cy.contains("Plugged In").should("be.visible");
    cy.contains("Map").should("be.visible");
    cy.contains("Debugging").should("be.visible");
    // cy.contains("Camera").should("be.visible");
    // cy.contains("Master Relay").should("be.visible");
    cy.contains("Cloud").should("be.visible");
    cy.contains("Relays").should("be.visible");
    cy.contains("Behavior").should("be.visible");
    cy.contains("Startup/Shutdown").should("be.visible");
    cy.contains("ROS Stopped").should("be.visible");
    cy.get("#startup-shutdown-card").contains("Start ROS").should("be.visible");
    cy.contains("Reset Robot Server").should("be.visible");
    // cy.contains("Unplug").should("be.visible");
    cy.contains("Robot Service Log").should("be.visible");
    // cy.contains("Video - Camera Off").should("be.visible");
    // cy.contains("Camera Off").should("be.visible");
    cy.contains("https://github.com/chrisl8/ArloBot").should("be.visible");
    cy.get("#settings").should("be.visible");
  });
}

function setSoundToQuiet() {
  it("should set sound to Quiet if currently set to Talk", () => {
    cy.contains("Behavior").click();
    cy.get("#talk-bequiet-button")
      .contains("Talk")
      .each(($elm) => {
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
}

function setSoundToTalk() {
  it("should set sound to Talk if currently set to Quiet", () => {
    cy.visit("");
    cy.contains("Behavior").click();
    cy.get("#talk-bequiet-button")
      .contains("Quiet")
      .each(($elm) => {
        cy.wrap($elm).then(() => {
          if ($elm.hasClass("brightly-negative-text")) {
            cy.get("#talk-bequiet-button").click();
          }
        });
      });

    cy.get("#talk-bequiet-button")
      .contains("Talk")
      .should("have.class", "brightly-positive-text");
  });
}

function setIdleToTimeout() {
  it("should set Idle to Timeout, not Never", () => {
    cy.contains("Behavior").click();
    cy.get("#idle-timeout-button")
      .contains("Never")
      .each(($elm) => {
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
}

module.exports = {
  resetRobotService,
  initialPageLoadItemsVisible,
  setSoundToQuiet,
  setIdleToTimeout,
  openPanelIfClosed,
  closePanelIfOpen,
};

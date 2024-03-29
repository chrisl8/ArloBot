# Cypress.io Tests

These tests literally test the robot, not just the web site. This has some repercussions, though:

* The tests are currently very specific to my implementation of the robot. Things like Relays will fail on a more generic or different robot.
* The order does matter, contrary to a proper Cypress.io setup.
* Some things are tested with ROS off and others with ROS on. The ROS on test set is sort of its own thing, since I expect the robot to MOVE!  
    * This means that these tests aren't good for an automated system like Travis.

## Test Order and Explanation

* Test files starting with `0` are basic site tests that can be run anytime. May even work without a proper robot.
* Test files starting with `1` are basic functional tests with ROS *OFF* that will not attempt to move the robot or engage the ROS features.
    * These features are centered around each panel. 
* Tests file starting with `2` are features that fall outside the `1` tests for various reasons, but still do not move the robot.
* Test files starting with `3` are full out ROS tests and may engage robot movement!
  
NOTE that each test file is supposed to run 100% by itself.  
This results in a lot of duplicate test code, but also means each test is easily self-contained, and we retest some of the more basic functions a lot, which is good.  

## Prerequisites

You may have to install some things on your Linux box to get Cypress to work:  
https://docs.cypress.io/guides/getting-started/installing-cypress#Linux-Prerequisites

## Usage
I suggest just running the tests one at a time starting with `0-1-...` until you finish them all.

### To start Cypress
```
cd cypress-tests
npx cypress open --e2e --browser=electron
```

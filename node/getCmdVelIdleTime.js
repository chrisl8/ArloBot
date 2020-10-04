const robotModel = require('./robotModel');

function getCmdVelIdleTime() {
  const dateNow = new Date();
  const lastActionDate = new Date(robotModel.lastMovementTime);
  const idleMinutes = (dateNow - lastActionDate) / 1000 / 60;

  return idleMinutes;
}

module.exports = getCmdVelIdleTime;

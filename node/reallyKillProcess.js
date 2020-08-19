// http://krasimirtsonev.com/blog/article/Nodejs-managing-child-processes-starting-stopping-exec-spawn
const psTree = require('ps-tree');

// eslint-disable-next-line func-names
module.exports = (pid, signal = 'SIGKILL', callback = function () {}) => {
  const killTree = true;
  if (killTree) {
    psTree(pid, (err, children) => {
      /** @namespace p.PID */
      [pid].concat(children.map((p) => p.PID)).forEach((tpid) => {
        try {
          process.kill(tpid, signal);
          // eslint-disable-next-line no-empty
        } catch (ex) {}
      });
      callback();
    });
  } else {
    try {
      process.kill(pid, signal);
      // eslint-disable-next-line no-empty
    } catch (ex) {}
    callback();
  }
};

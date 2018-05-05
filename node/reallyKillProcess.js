// http://krasimirtsonev.com/blog/article/Nodejs-managing-child-processes-starting-stopping-exec-spawn
const psTree = require('ps-tree');

module.exports = function(pid, signal, callback) {
  signal = signal || 'SIGKILL';
  callback = callback || function() {};
  const killTree = true;
  if (killTree) {
    psTree(pid, (err, children) => {
      [pid].concat(children.map((p) => p.PID)).forEach((tpid) => {
        try {
          process.kill(tpid, signal);
        } catch (ex) {}
      });
      callback();
    });
  } else {
    try {
      process.kill(pid, signal);
    } catch (ex) {}
    callback();
  }
};

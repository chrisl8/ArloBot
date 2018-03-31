async function wait(seconds) {
    return new Promise(resolve => {
        setTimeout(function () {
            resolve();
        }, seconds * 1000);
    });
}

module.exports = wait;

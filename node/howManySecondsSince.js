module.exports = function (oldDate) {
    if (oldDate !== null && oldDate !== undefined) {
        const rightNow = new Date();
        return (rightNow.getTime() - oldDate.getTime()) / 1000;
    } else {
        return 60 * 60 * 24 * 365; // Tell them it has been a year if it has never happened before.
    }
};

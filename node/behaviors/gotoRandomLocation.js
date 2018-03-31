const GotoRandomLocation = b3.Class(b3.Action);
GotoRandomLocation.prototype.name = 'GotoRandomLocation';
GotoRandomLocation.prototype.tick = function () { // Argument options: tick
    if (webModel.debugging) {
        console.log(this.name);
        webModelFunctions.scrollingStatusUpdate(this.name);
    }
    const repeatDelay = 60 * 10; // Ten minutes.
    // We have to have more than one destination. :) and the initial pose set.
    if (webModel.wayPoints.length > 1 && robotModel.initialPoseSet && howManySecondsSince(blackboard.get('lastRanTime', arloTree.id, arloTree.id)) >= repeatDelay) {
        console.log('GotoRandomLocation started:');
        console.log('Time since last call: ' + howManySecondsSince(blackboard.get('lastRanTime', arloTree.id, arloTree.id)));
        console.log('Previous destination: ' + blackboard.get('lastDestination', arloTree.id, arloTree.id));
        const destinationPicker = new Stochator({
            kind: "set",
            values: webModel.wayPoints
        });
        let destination;
        do {
            destination = destinationPicker.next();
        } while (destination === blackboard.get('lastDestination', arloTree.id, arloTree.id));
        // ^^^ To prevent going to the same place agian.
        blackboard.set('lastDestination', destination, arloTree.id, arloTree.id);
        blackboard.set('lastRanTime', new Date(), arloTree.id, arloTree.id);
        console.log('New Destination: ' + blackboard.get('lastDestination', arloTree.id, arloTree.id));
        console.log('--------------------------------');
        return b3.RUNNING;
    }
    return b3.FAILURE;

    // Place holder for now.
    //if (!webModel.pluggedIn) return b3.SUCCESS;
    //if (webModel.laptopFullyCharged) {
    //    if (webModel.unplugYourself) {
    //        if (!robotModel.unplugProcess.started) {
    //            webModel.status = 'Unplugging myself!';
    //            robotModel.unplugProcess.start();
    //            webModelFunctions.scrollingStatusUpdate(this.name + " process starting!");
    //            return b3.RUNNING;
    //        } else {
    //            // This should loop until the robot finishes unplugging itself.
    //            webModelFunctions.behaviorStatusUpdate(this.name + " unplugging . . .");
    //            return b3.RUNNING;
    //        }
    //    } else {
    //        if (!robotModel.unplugMeTextSent) {
    //            textme('Please unplug me!');
    //            robotModel.unplugMeTextSent = true;
    //            webModelFunctions.scrollingStatusUpdate(this.name + " requesting assistance.");
    //        }
    //        return b3.FAILURE;
    //    }
    //} else {
    //    webModel.status = 'Charging . . .';
    //    webModelFunctions.behaviorStatusUpdate(this.name + " waiting for full charge.");
    //    // We cannot do much else until we are unplugged.
    //    return b3.FAILURE;
    //}
};

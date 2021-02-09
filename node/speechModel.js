/*
 chatter happens every second, based on the results of Test,
 events happens when webModel is updated with the listed entry.

 repeatInterval - How often it will repeat this set of things to say
 spacing - How many seconds of nothing said from any entry must have elapsed before saying this entry.
 Test - Must be a function that returns true or false.

 repeat -  under "thingToSay" tells the randomizer whether to
    randomly pick from the entire list on every run,
    or to "mark them off" one by one and only pick from the remaining entries
    on each run until they are all gone and restart.
   If you say false, the "weight" doesn't mean much either, as it basically just indicates which one will be picked first.
 weight - Weight for how likely an entry is to be chosen.

 repeatInterval & spacing are typically 0 for events, as presumably you want it to chatter on every instance of an event, but maybe not always.
 Test only applies to chatter, as events are spoken when the event happens no matter what.

 Currently "chatter" just has the random noises,
 but in theory it could have different sections that only operate,
 under specific circumstances.
 Like: Only say these things while navigating to a waypoint,
 and only these when making a map,
 and only these when plugged in doing nothing.
 Etc.
 The "Test" should make that easy.

 Just remember that point in time event triggered speech need to happen in 'events', not 'chatter'.
 */

// TODO: SOME of these may be better off set within the feature that runs them,
//       in order to clearly only announce an action when it truly happens,
//       not just when it is requested.

// Received even when goal is set via Rviz:
// goalStatus: New Goal Set.
// goalStatus: Goal reached.

const speechModel = {
  chatter: {
    randomChatter: {
      repeatInterval: 600,
      spacing: 300,
      Test: () => {
        return true;
      },
      thingsToSay: {
        repeat: false,
        'CastleInTheSky-Ding': {
          text: '~/.arlobot/sounds/CastleInTheSky-Ding.wav',
          weight: 100,
        },
        'CastleInTheSky-Drip2': {
          text: '~/.arlobot/sounds/CastleInTheSky-Drip2.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep2a': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep2a.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep2b': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep2b.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep2c': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep2c.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep3a': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep3a.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep3b': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep3b.wav',
          weight: 100,
        },
        'CastleInTheSky-RobotBeep3c': {
          text: '~/.arlobot/sounds/CastleInTheSky-RobotBeep3c.wav',
          weight: 100,
        },
        'CastleInTheSky-Zap': {
          text: '~/.arlobot/sounds/CastleInTheSky-Zap.wav',
          weight: 100,
        },
      },
    },
  },
  events: {
    goalStatus: {
      repeatInterval: 10,
      spacing: 0,
      newGoalSet: {
        thingsToSay: {
          repeat: true,
          onMyWay: {
            text: 'On my way!',
            weight: 100,
          },
        },
      },
      goalReached: {
        thingsToSay: {
          repeat: true,
          iHaveArrived: {
            text: 'I have arrived.',
            weight: 100,
          },
          destinationReached: {
            text: 'Destination reached..',
            weight: 100,
          },
        },
      },
    },
    Escaping: {
      repeatInterval: 15,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          WallEwhoa: {
            text: '~/.arlobot/sounds/whoa.wav',
            weight: 100,
          },
        },
      },
    },
    ROSstart: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          ROSstart: {
            text: 'Starting ROS.',
            weight: 100,
          },
        },
      },
    },
    ROSisRunning: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          ROSisRunning: {
            text: 'ROS is running.',
            weight: 100,
          },
        },
      },
    },
    makeMap: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          makeMap: {
            text: 'Preparing to make map.',
            weight: 100,
          },
        },
      },
    },
    makeMapRunning: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          makeMapRunning: {
            text: 'Making Map.',
            weight: 100,
          },
        },
      },
    },
    loadMapRequested: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          loadMapRequested: {
            text: 'Loading map.',
            weight: 100,
          },
        },
      },
    },
    mapLoaded: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          mapLoaded: {
            text: 'Map Loaded.',
            weight: 100,
          },
        },
      },
    },
    haltRobot: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          WallEwhoa: {
            text: '~/.arlobot/sounds/whoa.wav',
            weight: 100,
          },
        },
      },
    },
    cameraOn: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          iSeeYou: {
            text: '~/.arlobot/sounds/I-see-you.wav',
            weight: 100,
          },
        },
      },
    },
    pluggedIn: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          thankYou: {
            text: 'Thank you for plugging me in.',
            weight: 100,
          },
        },
      },
      false: {
        thingsToSay: {
          repeat: true,
          depressed: {
            text: 'I suddenly feel very depressed.',
            weight: 10,
          },
          thankYou: {
            text: 'Thank you for unplugging me.',
            weight: 90,
          },
        },
      },
    },
    laptopFullyCharged: {
      repeatInterval: 0,
      spacing: 0,
      true: {
        thingsToSay: {
          repeat: true,
          True: {
            text: 'That was refreshing.',
            weight: 40,
          },
          True2: {
            text: "I'm ready to go!",
            weight: 60,
          },
        },
      },
      false: {
        thingsToSay: {
          repeat: true,
          unDone: {
            text: 'Starting to get tired.',
            weight: 100,
          },
        },
      },
    },
    mostRecentArrival: {
      repeatInterval: 0,
      spacing: 0,
      Here: {
        thingsToSay: {
          repeat: true,
          imHere: {
            text: "I'm here.",
            weight: 10,
          },
          yesMaster: {
            text: 'Yes Master?',
            weight: 90,
          },
        },
      },
      initial: {
        thingsToSay: {
          repeat: true,
          home: {
            text: 'Home sweet home.',
            weight: 60,
          },
          familiar: {
            text: 'This place looks vaguely familiar.',
            weight: 40,
          },
        },
      },
    },
  },
};

// Sting Keys cannot be set on object literals, so setting them here:
speechModel.events.goalStatus['New Goal Set.'] =
  speechModel.events.goalStatus.newGoalSet;
speechModel.events.goalStatus['Goal reached.'] =
  speechModel.events.goalStatus.goalReached;

module.exports = speechModel;

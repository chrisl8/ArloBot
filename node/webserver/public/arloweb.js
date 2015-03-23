/* https://github.com/chrisl8

   WARNING: Much of this code was unwittingly written by Google and Stack Overflow!

   ROS Info:
   http://robotwebtools.org/jsdoc/roslibjs/current/index.html
   http://wiki.ros.org/roslibjs/Tutorials/BasicRosFunctionality

   IMPLEMENTATION NOTES:
   This is meant to be called by index.php from the ArloWeb web control page.
   This requires the following code in the PHP file before calling/including this js file:
       <link rel="stylesheet" type="text/css" href="lcars.css" />
       <script type="text/javascript" src="jquery-2.1.1.min.js"></script>
       <link href="jquery-ui-1.10.4.custom.min.css" rel="stylesheet">
       <script src="jquery-ui-1.10.4.custom.min.js"></script>
       <script type="text/javascript" src="http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.min.js"></script>
       <script type="text/javascript" src="http://cdn.robotwebtools.org/roslibjs/current/roslib.min.js"></script>
       <script type="text/javascript" src="lcars.js"></script>
       </script>
   Without the above lines, this js file is incomplete.
   This was copied by hand, so if in doubt, see the index.php that should have come
   in the same folder with this file.
 */

var connectedToROS = false, // Track my opinion of the connection
    connectRequested = false, // For when we asked and are waiting patiently.
    pleaseWait = false, // Display Please Wait on the Connect button
    ros, // Empty global for actual connection.
    cmdVel, // Empty global for actual topic
    wakeScreen, // Empty global for actual topic
    toggleCamera, // Empty global for actual topic
    toggleRelay, // Empty global for actual topic
    sendTextToSpeak, // Empty global for actual topic
    shortDelay = 1000,
    longDelay = 3000,
    camera1On = false, // For tracking Camera status
    camera2On = false, // For tracking Camera status
    upperLightOn = false, // For tracking LED light bars
    lowerLightOn = false, // For tracking LED light bars
    UpperLightRowRelay = -1,
    UpperLightRowName = "TopLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LowerLightRowRelay = -1,
    LowerLightRowName = "BottomLightRow", // Found in arlobot_usbrelay/param/usbrelay.yaml
    RightMotorRelay = -1,
    RightMotorName = "RightMotor", // Found in arlobot_usbrelay/param/usbrelay.yaml
    LeftMotorRelay = -1,
    LeftMotorName = "LeftMotor"; // Found in arlobot_usbrelay/param/usbrelay.yaml

// Function to control "Action:" field text
var setActionField = function(newActionFieldText) {
    'use strict';
    $("#action").stop(true, true)
        .html(newActionFieldText)
        .fadeIn({
            duration: 0,
            queue: false
        })
        .fadeOut({
            duration: 5000,
            queue: false
        });
};

// TELEOP FUNCTIONS

//TODO: Pass arguments instead of using global variables?
// Global variables that can be set within the button functions
// and sent from called functions
var linear_speed = 0.0,
    angular_speed = 0.0,
    actionFieldText = "All Stop";

/* I need a function that will repeat the
   command periodically wile the drive buttons are down,
   Lest the robot stop, as it will NOT continue forever
   off of one command. It expects the command to repeat periodically,
   else it will get bored and stop. That is just how ROS works.
   Or perhaps it is a result of some setting, but ultimately it is best
   that it will quit if it doesn't get some updates from us periodically.

   http://stackoverflow.com/questions/15505272/javascript-while-mousedown
*/
var drivingButtonDownID = -1; // Global ID of mouse down interval

function sendTwistCommandToROS() { // We'll use the stop too!
    'use strict';
    if (connectedToROS && cmdVel !== undefined) {
        var twist = new ROSLIB.Message({
            linear: {
                x: linear_speed,
                y: 0.0,
                z: 0.0
            },
            angular: {
                x: 0.0,
                y: 0.0,
                z: angular_speed
            }
        });
        cmdVel.publish(twist); // cmdVel is defined in startROSfunctions()
        setActionField(actionFieldText);
    } else {
        setActionField("NOT Connected");
    }
}

function drivingButtonDown() {
    'use strict';
    // We need to run it once,
    // because setInterval waits the timeout before the first iteration!

    //Prevent multiple loops
    // http://conceptf1.blogspot.com/2014/01/javascript-triple-equals-vs-double-equals-operators.html
    if (drivingButtonDownID === -1) {
        sendTwistCommandToROS();
        // execute once per second
        drivingButtonDownID = setInterval(sendTwistCommandToROS, 1000);
    }
}

function drivingButtonUp() {
    'use strict';
    if (drivingButtonDownID !== -1) { // Only stop if exists/is moving
        linear_speed = 0.0;
        angular_speed = 0.0;
        clearInterval(drivingButtonDownID);
        // Stop the loop
        drivingButtonDownID = -1;
        // Allow new loops
        sendTwistCommandToROS();
        // Send stop Twist command to the robot
    }
}

// CONNECTION BUTTON Functions
/* This allows other events,
   besides hovering and pressing
   to update the connect button's appearance.
*/
var connectButtonHovered = false;

var updateConnectedButton = function() {
    'use strict';
    var $connectButton = $('#connectButton-li');
    if (pleaseWait) {
        $('span#connectText').html("PLEASE WAIT");
        $connectButton.css('background-color', '#BBAA55');
        $connectButton.css('letter-spacing', '1px');
    } else if (connectedToROS) {
        if (connectButtonHovered) {
            $connectButton.css('background-color', '#882211');
            $('span#connectText').html("STOP ROS?");
        } else {
            $('span#connectText').html("CONNECTED");
            $connectButton.css('background-color', '#3366FF');
            $connectButton.css('letter-spacing', '1px');
        }
    } else {
        if (connectButtonHovered) {
            $connectButton.css('background-color', '#5599FF');
            $('span#connectText').html("START ROS?");
        } else {
            $('span#connectText').html("NO CONNECTION");
            $connectButton.css('background-color', '#FF9900');
            $connectButton.css('letter-spacing', '0');
        }
    }
};

// BUTTON DEFINITIONS using jQuery and the LCARS functions
// http://learn.jquery.com/using-jquery-core/document-ready/
$(document).ready(function() {
    'use strict';

    // CONNECTION BUTTON
    $('#connectButton-li').on("mousedown touchstart", function() {
            if (pleaseWait) {
                $(this).addClass("pressOnButton");
                updateConnectedButton();
            } else if (connectedToROS) {
                $(this).removeClass("lightUpButton");
                $(this).addClass("pressOnButton");
                $.get('stopROS.php', function(data) {
                    setActionField(data);
                });
                pleaseWait = true;
                setTimeout(setConnectRequestedFalse, 30000);
                updateConnectedButton();
            } else {
                $(this).removeClass("lightUpButton");
                $(this).addClass("pressOnButton");
                $.get('startROS');
                console.log("tail -f /opt/lampp/logs/error_log # To see what ROS is doing.");
                setActionField("ROS Startup");
                pleaseWait = true;
                setTimeout(setConnectRequestedFalse, 45000);
                updateConnectedButton();
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
            updateConnectedButton();
        })
        .hover(function() {
            connectButtonHovered = true;
            updateConnectedButton();
        }, function() {
            $(this).removeClass("lightUpButton");
            connectButtonHovered = false;
            updateConnectedButton();
        });

    // TELEOP LCARS BUTTONS
    // Forward
    var $forwardButton = $('a.forwardButton');
    $forwardButton.lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'left', // left or right
            text: '<sub>&#9650;</sub>' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });
    $forwardButton.hover(function() {
            $(this).find('span').html('<sup>&#9650;</sup>');
        }, function() {
            $(this).find('span').html("<sub>&#9650;</sub>");
        })
        // http://stackoverflow.com/questions/3303469/does-mousedown-mouseup-in-jquery-work-for-the-ipad
        .on("mousedown touchstart", function() {
            $(this).css('background-color', '#000088');
            linear_speed = $("#travelSpeedSlider").slider("value");
            angular_speed = 0.0;
            // Straight, no rotation
            actionFieldText = "Forward";
            drivingButtonDown(); // Has to be a loop or it will get bored and stop
        })
        .on("mouseup mouseout touchend", function() {
            actionFieldText = "All Stop";
            drivingButtonUp(); // Stop the loop, which will also stop the robot.
            $(this).css('background-color', '#5599FF');
        });
    // Left
    $('a.leftButton').lcarsButton({
        rounded: 'left', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'left', // left or right
            text: '-<' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });
    $('a.leftButton').hover(function() {
            $(this).find('span').html('<-');
        }, function() {
            $(this).find('span').html("-<");
        })
        .on("mousedown touchstart", function() {
            $(this).css('background-color', '#000088');
            linear_speed = 0.0; // Rotate in place, no liner movement
            angular_speed = $("#rotateSpeedSlider").slider("value");
            actionFieldText = "Left";
            drivingButtonDown(); // Has to be a loop or it will get bored and stop
        })
        .on("mouseup mouseout touchend", function() {
            actionFieldText = "All Stop";
            drivingButtonUp(); // Stop the loop, which will also stop the robot.
            $(this).css('background-color', '#5599FF');
        });
    // Stop
    $('a.stopButton').lcarsButton({
        rounded: 'none', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'tan',
        subTitle: { // The sub title for your button
            direction: 'left', // left or right
            text: '----' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });
    $('a.stopButton').hover(function() {
            $(this).find('span').text('|');
        }, function() {
            $(this).find('span').text("----");
        })
        .on("mousedown touchstart", function() {
            $(this).css('background-color', '#882211');
            /* There may be need to send a steady stream
             of BE STILL messages, so here it is,
             though the interval may be insufficient to avoid fighting
             with somebody. */
            linear_speed = 0.0;
            angular_speed = 0.0;
            actionFieldText = "All Stop";
            drivingButtonDown(); // Has to be a loop or it will get bored and stop
        })
        .on("mouseup mouseout touchend", function() {
            actionFieldText = "All Stop";
            drivingButtonUp(); // Stop the loop, which will also stop the robot.
            $(this).css('background-color', '#BBAA55');
        });
    // Right
    $('a.rightButton').lcarsButton({
        rounded: 'right', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'right', // left or right
            text: '>-' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });
    $('a.rightButton').hover(function() {
            $(this).find('span').html('->');
        }, function() {
            $(this).find('span').html(">-");
        })
        .on("mousedown touchstart", function() {
            $(this).css('background-color', '#000088');
            linear_speed = 0.0; // Rotate in place, no liner movement
            angular_speed = -$("#rotateSpeedSlider").slider("value");
            actionFieldText = "Right";
            drivingButtonDown(); // Has to be a loop or it will get bored and stop
        })
        .on("mouseup mouseout touchend", function() {
            actionFieldText = "All Stop";
            drivingButtonUp(); // Stop the loop, which will also stop the robot.
            $(this).css('background-color', '#5599FF');
        });
    // Reverse
    $('a.reverseButton').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'left', // left or right
            text: '<sup>&#9660;</sup>' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });
    $('a.reverseButton').hover(function() {
            $(this).find('span').html('<sub>&#9660;</sub>');
        }, function() {
            $(this).find('span').html("<sup>&#9660;</sup>");
        })
        .on("mousedown touchstart", function() {
            $(this).css('background-color', '#000088');
            linear_speed = -$("#travelSpeedSlider").slider("value");
            angular_speed = 0.0; // Straight, no rotation
            actionFieldText = "Reverse";
            drivingButtonDown(); // Has to be a loop or it will get bored and stop
        })
        .on("mouseup mouseout touchend", function() {
            actionFieldText = "All Stop";
            drivingButtonUp(); // Stop the loop, which will also stop the robot.
            $(this).css('background-color', '#5599FF');
        });

    // LOWER SIDE PANEL BUTTONS
    // PING Button:
    $('#pingButton-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            $.get('PING', function(data) {
                setActionField(data);
            });
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // This will "highlight" the buttons on the side panel upon hover,
        // Which I think is important since the cursor change is off
        // I am using addClass instead of setting the background-color directly,
        // Since I don't know what it was (it is different on every other button,
        // So I don't know what to set it back to.
        // Note you have to use !important for the class background-color
        // to override the element style
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // wakeScreenButton:

    $('#wakeScreenButton-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            if (connectedToROS && wakeScreen !== undefined) {
                // wakeScreen is defined in startROSfunctions()
                var wakeScreenRequest = new ROSLIB.ServiceRequest({});
                // We have to send a request, even if it is empty, or we get errors
                //wakeScreen.callService(); // This causes errors.
                wakeScreen.callService(wakeScreenRequest, function(result) {
                    if (result.toggleSuccess) {
                        setActionField("Screen Woken");
                    } else {
                        setActionField("Wake Screen FAILED");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // Microphone Button 1:

    var microphoneOn = false;

    $('#empty1Button-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            //console.log("EMPTY Button 1");
            if (microphoneOn) {
                $('#microphone').css('display', 'none');
                microphoneOn = false;
                //$('a.vncButton').find('span').html('off');
                setActionField("Microphone OFF");
            } else {
                $('#microphone').css('display', 'block');
                microphoneOn = true;
                //$('a.vncButton').find('span').html('ON');
                setActionField("Microphone ON");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // Empty Button 2:

    $('#empty2Button-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            console.log("EMPTY Button 2");
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // Empty Butotn 3:

    $('#empty3Button-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            console.log("EMPTY Button 3");
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // STAYCLOSE Button:
    $('#staycloseButton-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            $.get('STAYCLOSE', function(data) {
                setActionField(data);
            });
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // RESET Button:
    $('#resetButton-li').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            $.get('RESET', function(data) {
                setActionField(data);
            });
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        })
        // Highlight
        .hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        });

    // MAIN VIEWSCREEN BUTTONS

    // Camera 1 Button
    $('a.camera1Button').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'right', // left or right
            text: '-' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });

    $('a.camera1Button').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            if (connectedToROS && toggleCamera !== undefined) {
                var toggleCameraRequest = new ROSLIB.ServiceRequest({
                    camera: 'c519', // args from rosservice info <service>
                    state: true // Note javaScript uses true not True for bool
                });

                if (camera1On === true) {
                    toggleCameraRequest.state = false;
                    // Already set to true in declaration, so only need to test for one side
                }
                // toggleCamera is defined in startROSfunctions
                toggleCamera.callService(toggleCameraRequest, function(result) {
                    if (result.toggleSuccess) {
                        setActionField("Camera 1 Switched");
                    } else {
                        setActionField("Camera 1 Failed");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        });

    // Camera 2 button
    $('a.camera2Button').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'right', // left or right
            text: '-' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });

    $('a.camera2Button').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            if (connectedToROS && toggleCamera !== undefined) {
                var toggleCameraRequest = new ROSLIB.ServiceRequest({
                    camera: 'laptop', // args from rosservice info <service>
                    state: true // Note javaScript uses true not True for bool
                });

                if (camera2On === true) {
                    toggleCameraRequest.state = false;
                    // Already set to true in declaration, so only need to test for one side
                }
                // toggleCamera is defined in startROSfunctions
                toggleCamera.callService(toggleCameraRequest, function(result) {
                    if (result.toggleSuccess) {
                        setActionField("Camera 2 Switched");
                    } else {
                        setActionField("Camera 2 Failed");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        });

    // Upper Light Button
    $('a.upperLightButton').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'right', // left or right
            text: '-' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });

    $('a.upperLightButton').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            if (connectedToROS && toggleRelay !== undefined) {
                var toggleLightRequest = new ROSLIB.ServiceRequest({
                    relay: UpperLightRowName, // args from rosservice info <service>
                    state: true // Note javaScript uses true not True for bool
                });
                if (upperLightOn) {
                    toggleLightRequest.state = false;
                    // Already set to true in declaration, so only need to test for one side
                }
                // toggleRelay is defined in startROSfunctions()
                toggleRelay.callService(toggleLightRequest, function(result) {
                    if (result.toggleSuccess) {
                        upperLightOn = toggleLightRequest.state;
                        if (upperLightOn) {
                            $('a.upperLightButton').find('span').html('ON');
                            setActionField("Upper Light ON");
                        } else {
                            $('a.upperLightButton').find('span').html('off');
                            setActionField("Upper Light off");
                        }
                    } else {
                        setActionField("Upper Light failed");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        });

    // Lower Light button
    $('a.lowerLightButton').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: true, // this is true or false
        color: 'lightBlue',
        subTitle: { // The sub title for your button
            direction: 'right', // left or right
            text: '-' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });

    $('a.lowerLightButton').on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            if (connectedToROS && toggleRelay !== undefined) {
                var toggleLightRequest = new ROSLIB.ServiceRequest({
                    relay: LowerLightRowName, // args from rosservice info <service>
                    state: true // Note javaScript uses true not True for bool
                });
                if (lowerLightOn) {
                    toggleLightRequest.state = false;
                    // Already set to true in declaration, so only need to test for one side
                }
                // toggleRelay is defined in startROSfunctions()
                toggleRelay.callService(toggleLightRequest, function(result) {
                    if (result.toggleSuccess) {
                        lowerLightOn = toggleLightRequest.state;
                        if (lowerLightOn) {
                            $('a.lowerLightButton').find('span').html('ON');
                            setActionField("Lower Light ON");
                        } else {
                            $('a.lowerLightButton').find('span').html('off');
                            setActionField("Lower Light off");
                        }
                    } else {
                        setActionField("Lower Light failed");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        });

    // TODO: Should be pulled from somewhere else?
    var maxTravelSpeed = 1.0;
    var maxRotateSpeed = 4.0;

    // SPEED SLIDERS
    $(function() {
        var initialTravelValue = 0.1,
            initialRotateValue = 1.0;

        $("#travelSpeedSlider").slider({
            orientation: "horizontal",
            range: "min",
            max: maxTravelSpeed,
            step: 0.1,
            value: initialTravelValue, // starting value
            slide: function(event, ui) {
                $('span#travelSpeed').html(ui.value);
            }
        });
        $("#rotateSpeedSlider").slider({
            orientation: "horizontal",
            range: "min",
            max: maxRotateSpeed,
            step: 0.1,
            value: initialRotateValue, // starting value
            slide: function(event, ui) {
                $('span#rotateSpeed').html(ui.value);
            }
        });
        $('span#travelSpeed').html(initialTravelValue);
        $('span#rotateSpeed').html(initialRotateValue);
    });

    // SPEAK FUNCTION

    var textToSpeak = "I have nothing to say.";

    // http://stackoverflow.com/questions/155188/trigger-a-button-click-with-javascript-on-the-enter-key-in-a-text-box
    $("#speak").keyup(function(event) {
        //console.log(event.keyCode); // Great debugging! ;)
        if (event.keyCode === 13) {
            $("#speakButton").mousedown();
            $("#speakButton").mouseup();
        }
    });

    // SPEAK BUTTON
    $('a#speakButton').lcarsButton({
        rounded: 'both', // accepts both, left, right, none
        extended: false, // this is true or false
        color: 'orange',
        subTitle: { // The sub title for your button
            direction: 'left', // left or right
            text: '' // the text for the sub title
        },
        blank: 'none' // blank button? left / right / none
    });

    $('#speakButton').hover(function() {
            $(this).addClass("lightUpButton");
        }, function() {
            $(this).removeClass("lightUpButton");
        })
        .on("mousedown touchstart", function() {
            $(this).addClass("pressOnButton");
            textToSpeak = document.getElementsByName('speak')[0].value;
            console.log(textToSpeak);
            if (connectedToROS && sendTextToSpeak !== undefined) {
                var speakRequest = new ROSLIB.ServiceRequest({
                    text_to_speak: textToSpeak // args from rosservice info <service>
                });
                // sendTextToSpeak is defined in startROSfunctions()
                sendTextToSpeak.callService(speakRequest, function(result) {
                    if (result.speach_result) {
                        setActionField("Text Spoken");
                    } else {
                        setActionField("Speech failure");
                    }
                });
            } else {
                setActionField("NOT Connected");
            }
            document.getElementsByName('speak')[0].value = '';
        })
        .on("mouseup mouseout touchend", function() {
            $(this).removeClass("pressOnButton");
        });
    // END Button Definitions here.

    // JOYSTICK on VIDEO IMAGE
    //console.log("touchscreen is", VirtualJoystick.touchScreenAvailable() ? "available" : "not available");
    var joystick = new VirtualJoystick({
        container: document.getElementById('joyStickBox'),
        mouseSupport: false,
        limitStickTravel: true
    });
    //var outputEl    = document.getElementById('result');
    joystick.addEventListener('touchStart', function() {
        console.log('Joystick Start');
        //outputEl.innerHTML  = '<b>Active:</b>';
    });
    joystick.addEventListener('touchEnd', function() {
        console.log('Joystick End');
        //outputEl.innerHTML  = '<b>In</b>active.';
    });
    joystick.addEventListener('touchMove', function() {
        console.log('dx:' + joystick.deltaX().toFixed(0) + ' dy:' + joystick.deltaY().toFixed(0) + ' linear: ' + (-joystick.deltaY().toFixed(0) / 100) * maxTravelSpeed + ' rotate: ' + (joystick.deltaX().toFixed(0) / 100) * maxRotateSpeed
                // + ' '
                // + (joystick.right() ? ' right'  : '')
                // + (joystick.up()    ? ' up'     : '')
                // + (joystick.left()  ? ' left'   : '')
                // + (joystick.down()  ? ' down'   : '')
            );
            //     outputEl.innerHTML  = '<b>Active:</b> '
            //     + ' dx:'+joystick.deltaX().toFixed(0)
            //     + ' - '
            //     + ' dy:'+joystick.deltaY().toFixed(0)
            //     + '<br/>Direction:'
            //     + (joystick.right() ? ' right'  : '')
            //     + (joystick.up()    ? ' up'     : '')
            //     + (joystick.left()  ? ' left'   : '')
            //     + (joystick.down()  ? ' down'   : '');
    });

}); // END Document Ready Section here.

var closeDeadROSConnection = function() {
    'use strict';
    console.log("Closing dead ROS connection.");
    if (ros !== undefined) {
        ros.close();
    }
    console.log("CLOSED dead ROS connection!");
};

var checkROSService = function(serviceResult) {
    'use strict';
    //console.log(serviceResult);
    if (serviceResult === -1) {
        return false;
    }
    return true;
};

var subscribeToMetatron_idStatus = function() {
    'use strict';
    // This should serve as a template for all service subscriptions
    // Make sure we are still connected.
    // No need to recall myself as a new connect will do that.
    if (!connectedToROS) {
        return;
    }
    // Make sure service exists:
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
    ros.getTopics(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        if (!checkROSService(result.indexOf('/metatron_id/status'))) {
            setTimeout(subscribeToMetatron_idStatus, longDelay);
            // Try again when all topics are up!
            return;
        }

        // THIS is where you put the subscription code:

        setActionField("Metatron subscription");
        var metatron_idStatus = new ROSLIB.Topic({
            ros: ros,
            name: '/metatron_id/status', // Obtain name by running 'rostopic list'
            messageType: 'metatron_services/metatron_status' // Obtain Type by running 'rostopic info <name>'
        }); // Obtain message.??? by running 'rosmsg show <messageType>'

        metatron_idStatus.subscribe(function(message) {
            //console.log('Received message on ' + usbStatus.name + ': ' + message.relayPresent);
            //console.log(message.relayOn);
            // usbStatus.unsubscribe();
            $("#statusLight").toggle("fade");
            var img = document.getElementById("videoFeed");
            // Status light. :)
            if (message.camera1) {
                if (camera1On === false) {
                    // This will cause the image in img id="videoFeed" to refresh,
                    // by setting the source again.
                    img.src = "http://" + location.hostname + ":58180/?action=stream";
                    // see var img declaration above!
                }
                camera1On = true;
                //$('span#camera1').html("Camera 1 ON");
                $('a.camera1Button').find('span').html('ON');
            } else if (!message.camera1) {
                if (camera1On === true && message.camera2 === false) {
                    img.src = "http://" + location.host + "/arloweb/xscreen.png";
                }
                camera1On = false;
                //$('span#camera1').html("Camera 1 OFF");
                $('a.camera1Button').find('span').html('off');
            }

            if (message.camera2) {
                if (camera2On === false) {
                    img.src = "http://" + location.hostname + ":58180/?action=stream";
                }
                camera2On = true;
                //$('span#camera2').html("Camera 2 ON");
                $('a.camera2Button').find('span').html('ON');
            } else if (!message.camera2) {
                if (camera2On === true && message.camera1 === false) {
                    img.src = "http://" + location.host + "/arloweb/xscreen.png";
                }
                camera2On = false;
                //$('span#camera2').html("Camera 2 OFF");
                $('a.camera2Button').find('span').html('off');
            }

            if (message.newScreenShot && !message.camera1 && !message.camera2) {
                /* Force reload:http://stackoverflow.com/questions/1077041/refresh-image-with-a-new-one-at-the-same-url
                 Not cache friendly, but the throttling deals with that anyway.
                 */
                img.src = "http://" + location.host + "/arloweb/xscreen.png?" + new Date().getTime();
            }
        });
    });
};

var subscribeToUsbRelayStatus = function() {
    'use strict';
    // Make sure we are still connected.
    // No need to recall myself as a new connect will do that.
    if (!connectedToROS) {
        return;
    }
    // Make sure service exists:
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
    ros.getTopics(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        if (!checkROSService(result.indexOf('/arlobot_usbrelay/usbRelayStatus'))) {
            setTimeout(subscribeToUsbRelayStatus, longDelay); // Try MYSELF again when all topics are up!
            return;
        }

        // THIS is where you put the subscription code:

        setActionField("Relay subscription");
        var arlobot_usbrelayStatus = new ROSLIB.Topic({
            ros: ros,
            name: '/arlobot_usbrelay/usbRelayStatus', // rostopic list
            messageType: 'arlobot_msgs/usbRelayStatus' // rostopic info <topic>
        });

        arlobot_usbrelayStatus.subscribe(function(message) {
            $("#statusLight2").toggle("fade");
            // Status light. :)
            //console.log(message); // for debugging
            if (message.relayOn[UpperLightRowRelay - 1]) { // zero based array
                upperLightOn = true;
                // for tracking
                $('a.upperLightButton').find('span').html('ON');
            } else {
                upperLightOn = false;
                $('a.upperLightButton').find('span').html('off');
            }
            if (message.relayOn[LowerLightRowRelay - 1]) { // zero based array
                lowerLightOn = true;
                // for tracking
                $('a.lowerLightButton').find('span').html('ON');
            } else {
                lowerLightOn = false;
                // for tracking
                $('a.lowerLightButton').find('span').html('off');
            }
            if (message.relayOn[LeftMotorRelay - 1]) { // zero based array
                $('span#leftMotor').html("ON");
            } else {
                $('span#leftMotor').html("off");
            }
            if (message.relayOn[RightMotorRelay - 1]) { // zero based array
                $('span#rightMotor').html("ON");
            } else {
                $('span#rightMotor').html("off");
            }
        });
    });
};

var subscribeToArlo_status = function() {
    'use strict';
    // Make sure we are still connected.
    // No need to recall myself as a new connect will do that.
    if (!connectedToROS) {
        return;
    }
    // Make sure service exists:
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
    ros.getTopics(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        if (!checkROSService(result.indexOf('/arlo_status'))) {
            setTimeout(subscribeToArlo_status, longDelay); // Try MYSELF again when all topics are up!
            return;
        }

        // THIS is where you put the subscription code:

        setActionField("Arlo_status subscription");
        var arlobot_arlo_status = new ROSLIB.Topic({
            ros: ros,
            name: '/arlo_status', // rostopic list
            messageType: 'arlobot_msgs/arloStatus' // rostopic info <topic>
        });

        arlobot_arlo_status.subscribe(function(message) {
            //console.log(message); // for debugging
            if (message.safeToProceed === true) {
                $('span#safeToProceed').html("True").css('color', '#5599ff');
            } else {
                $('span#safeToProceed').html("False").css('color', '#BB4411');
            }
            if (message.safeToRecede === true) {
                $('span#safeToRecede').html("True").css('color', '#5599ff');
            } else {
                $('span#safeToRecede').html("False").css('color', '#BB4411');
            }
            if (message.Escaping === true) {
                $('span#Escaping').html("True").css('color', '#BB4411');
            } else {
                $('span#Escaping').html("False").css('color', '#5599ff');
            }
            // TODO: Highlight/color if the diff is greater than X?
            $('span#Heading').html((message.Heading).toFixed(3));
            $('span#gyroHeading').html((message.gyroHeading).toFixed(3));
            if (message.abd_speedLimit < 11) {
                $('span#abd_speedLimit').html(message.abd_speedLimit).css('color', '#BB4411');
            } else if (message.abd_speedLimit < 100) {
                $('span#abd_speedLimit').html(message.abd_speedLimit).css('color', '#FF9900');
            } else {
                $('span#abd_speedLimit').html(message.abd_speedLimit).css('color', '#5599ff');
            }
            if (message.abdR_speedLimit < 11) {
                $('span#abdR_speedLimit').html(message.abdR_speedLimit).css('color', '#BB4411');
            } else if (message.abdR_speedLimit < 100) {
                $('span#abdR_speedLimit').html(message.abdR_speedLimit).css('color', '#FF9900');
            } else {
                $('span#abdR_speedLimit').html(message.abdR_speedLimit).css('color', '#5599ff');
            }
            $('span#minDistanceSensor').html(message.minDistanceSensor);
            $('span#robotBatteryLevel').html((message.robotBatteryLevel).toFixed(1) + "v");
            if (message.robotBatteryLevel > 14) {
                $('span#robotBatteryLevel').css('color', '#007927');
            } else if (message.robotBatteryLevel > 13) {
                $('span#robotBatteryLevel').css('color', '#FF9900');
            } else if (message.robotBatteryLevel < 12) {
                $('span#robotBatteryLevel').css('color', '#BB4411');
            }
            if (message.leftMotorPower) {
                $('span#leftMotorPower').html("Yes");
            } else {
                $('span#leftMotorPower').html("no");
            }
            if (message.rightMotorPower) {
                $('span#rightMotorPower').html("Yes");
            } else {
                $('span#rightMotorPower').html("no");
            }
            $('span#laptopBatteryPercent').html(message.laptopBatteryPercent + "%");
            if (message.laptopBatteryPercent === 100) {
                $('span#laptopBatteryPercent').css('color', '#007927');
            } else if (message.laptopBatteryPercent < 50) {
                $('span#laptopBatteryPercent').css('color', '#FF9900');
            }
            if (message.laptopBatteryPercent < 20) {
                $('span#laptopBatteryPercent').css('color', '#BB4411');
            }
            if (message.acPower) {
                $('span#acPower').html("Yes").css('color', '#007927');
            } else {
                $('span#acPower').html("no").css('color', '#5599ff');
            }
        });
    });
};

var getUsbRelayNumbers = function() {
    'use strict';
    // Make sure we are still connected.
    // No need to recall myself as a new connect will do that.
    if (!connectedToROS) {
        return;
    }
    // Make sure service exists:
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, longDelay);
    ros.getServices(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        if (!checkROSService(result.indexOf('/arlobot_usbrelay/find_relay'))) {
            setTimeout(getUsbRelayNumbers, longDelay); // Try MYSELF again when all topics are up!
            return;
        }

        // THIS is where you put the subscription code:

        setActionField("Getting Relays");
        var findRelay = new ROSLIB.Service({
            ros: ros,
            name: '/arlobot_usbrelay/find_relay',
            serviceType: 'arlobot_msgs/FindRelay'
        });

        var request0 = new ROSLIB.ServiceRequest({
            relay: UpperLightRowName // Found in arlobot_usbrelay/param/usbrelay.yaml
        });

        var getRelay0 = function() {
            findRelay.callService(request0, function(result) {
                UpperLightRowRelay = result.relayNumber;
                //console.log("Relay request SUCCESS!");
            }, function() {
                //console.log("Relay request FAILED!");
                setTimeout(getRelay0, shortDelay);
            });
        };
        getRelay0();

        var request1 = new ROSLIB.ServiceRequest({
            relay: LowerLightRowName // Found in arlobot_usbrelay/param/usbrelay.yaml
        });

        var getRelay1 = function() {
            findRelay.callService(request1, function(result) {
                LowerLightRowRelay = result.relayNumber;
                //console.log("Relay request SUCCESS!");
            }, function() {
                //console.log("Relay request FAILED!");
                setTimeout(getRelay1, shortDelay);
            });
        };
        getRelay1();

        var request2 = new ROSLIB.ServiceRequest({
            relay: RightMotorName // Found in arlobot_usbrelay/param/usbrelay.yaml
        });

        var getRelay2 = function() {
            findRelay.callService(request2, function(result) {
                RightMotorRelay = result.relayNumber;
                //console.log("Relay request SUCCESS!");
            }, function() {
                //console.log("Relay request FAILED!");
                setTimeout(getRelay2, shortDelay);
            });
        };
        getRelay2();

        var request3 = new ROSLIB.ServiceRequest({
            relay: LeftMotorName // Found in arlobot_usbrelay/param/usbrelay.yaml
        });

        var getRelay3 = function() {
            findRelay.callService(request3, function(result) {
                LeftMotorRelay = result.relayNumber;
                //console.log("Relay request SUCCESS!");
            }, function() {
                //console.log("Relay request FAILED!");
                setTimeout(getRelay3, shortDelay);
            });
        };
        getRelay3();
    });
};

var startROSfunctions = function() {
    'use strict';
    setActionField("Starting functions");

    // These have to be set up AFTER ROS is working!

    cmdVel = new ROSLIB.Topic({ // Setup ROS cmdVel Publishing a Topic for the TeleOp function
        ros: ros,
        name: '/cmd_vel_mux/input/web',
        messageType: 'geometry_msgs/Twist'
    });

    // The existence of these services was checked in checkROSServices() already
    wakeScreen = new ROSLIB.Service({
        ros: ros,
        name: '/metatron_id/wake_screen', // rosservice list
        serviceType: 'metatron_services/WakeScreen' // rosservice info <service>
    });

    toggleCamera = new ROSLIB.Service({
        ros: ros,
        name: '/metatron_id/toggle_camera', // rosservice list
        serviceType: 'metatron_services/ToggleCamera' // rosservice info <service>
    });

    toggleRelay = new ROSLIB.Service({
        ros: ros,
        name: '/arlobot_usbrelay/toggle_relay', // rosservice list
        serviceType: 'arlobot_msgs/ToggleRelay' // rosservice info <service>
    });

    sendTextToSpeak = new ROSLIB.Service({
        ros: ros,
        name: '/metatron_speaker', // rosservice list
        serviceType: 'metatron_services/SpeakText' // rosservice info <service>
    });

    // Each topic function will do its own checking to see if the topic is live or not.
    setTimeout(subscribeToMetatron_idStatus, shortDelay); // Start with a slight delay
    setTimeout(subscribeToUsbRelayStatus, shortDelay * 2); // Increase delay with each to spread them out.
    setTimeout(subscribeToArlo_status, shortDelay * 3);
    setTimeout(getUsbRelayNumbers, shortDelay * 4); // Start Service request functions too
};

var checkROSServices = function() { // Check for all of the VITAL startups
    'use strict';
    setActionField("Checking services");
    //console.log("Getting Service List:");
    var closeDeadConnectionTime;
    closeDeadConnectionTime = setTimeout(closeDeadROSConnection, shortDelay);
    ros.getServices(function(result) { // Unfortunately this can stall with no output!
        clearTimeout(closeDeadConnectionTime);
        setActionField("Checking topics");
        if (!checkROSService(result.indexOf("/rosapi/topics"))) { // This is the first service we need always!
            setTimeout(checkROSServices, longDelay);
            return;
        }
        // Check for all of the services we need before moving on!
        setActionField("Checking wake_screen");
        if (!checkROSService(result.indexOf('/metatron_id/wake_screen'))) {
            setTimeout(checkROSServices, longDelay);
            return;
        }
        setActionField("Checking toggle_camera");
        if (!checkROSService(result.indexOf('/metatron_id/toggle_camera'))) {
            setTimeout(checkROSServices, longDelay);
            return;
        }
        setActionField("Checking toggle_relay");
        if (!checkROSService(result.indexOf('/arlobot_usbrelay/toggle_relay'))) {
            setTimeout(checkROSServices, longDelay);
            return;
        }
        setActionField("Checking metatron_speaker");
        if (!checkROSService(result.indexOf('/metatron_speaker'))) {
            setTimeout(checkROSServices, longDelay);
            return;
        }
        setActionField("Checking find_relay");
        if (!checkROSService(result.indexOf('/arlobot_usbrelay/find_relay'))) {
            setTimeout(checkROSServices, longDelay);
            return;
        }
        //console.log("OK to go!");
        connectedToROS = true;
        setActionField("Connected!");
        updateConnectedButton();
        // Now we feel we have a connection!
        startROSfunctions();
    });
};

var setConnectRequestedFalse = function() {
    'use strict';
    connectRequested = false;
    pleaseWait = false;
    updateConnectedButton();
};

var pollROS = function() {
    'use strict';
    connectedToROS = false;

    ros = new ROSLIB.Ros({
        url: 'ws://' + location.hostname + ':9090'
    });

    ros.on('connection', function() {
        setActionField('Websocket connected.');
        //connectRequested = true;
        updateConnectedButton();
        checkROSServices();
    });

    ros.on('error', function(error) {
        //console.log('Error connecting to websocket server: ', error);
        setActionField('Websocket eror');
        if (ros !== undefined) {
            ros.close();
        }
    });

    ros.on('close', function() {
        //console.log('Connection to websocket server closed.');
        setActionField('Websocket closed');
        connectedToROS = false;
        updateConnectedButton();
        setTimeout(pollROS, shortDelay);
    });
};

pollROS();

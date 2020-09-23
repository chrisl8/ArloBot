# Simple Test List for all Functions

0. Install on Virtual Machine:   
    a. Set up a VM with a minimal install of Ubuntu.  
    b. Ideally run Arlobot install from the web, but you may have to install open-sshd first, and install locally if you want to test before pushing.  
    c. Robot web site should come up in Firefox on installed system when install is done.
     - Make sure it functions.
     
1. Basic Robot Operation on Three Flower

    Open terminal and run:
    - start-robot.sh
        - Does the XV11 spin up?
        - Does the Sweep spin up?
        - Can you see the following output in RVIZ on their topic that isn't /scan?
            - XV11
            - Scanse Sweep
            - Kinect
            - ASUS Xtion

1. Cypress

    a. Ensure that the web interface will point to your test install.
        Edit website/src/container/App.js as required.

    b. "Can Cypress run?"

    ```shell script
    cd ~/catkin_ws/src/ArloBot/cypress-tests
    npx cypress open
    ```  

    c. Run first test on non-robot.

    d. Run all tests on Robot.

2. Basic Website and Robot Usage

 - Open a terminal and run: `pm2 flush;pm2 log` where you should see the Web Interface log.
 - Open the Web Interface and test every feature.
 
3. Scripts

 - Test every script in the scripts folder
 
4. Mycroft

 - Ensure robot responds to basic requests
    - Weather
    - Time
 - Test each voice command
    - 'start ros' or 'start robot'
    - TODO: Add the new ones here.

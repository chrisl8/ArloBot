# Behavior planning #

This is a description of the behavior tree I am using for the robot.
If you paste the contents of arloTreeData.json into the "Import Tree"
function of http://behavior3js.guineashots.com/editor/ you can see
the tree.

## Tree structure ##

    . Robot Sequence # A root sequence for the entire tree
    ├──Poll Node    # A node at the top to perform any required polling functions.
    ├──Startup Sequence # A Sequence to track all functions required to get the robot up and running fully.
    |   ├──StartROS Node        # Wait for start ROS request.
    |   ├──Priority             # Put Explore vs. LaoadMpa into a priority
    |   |   ├──AutoExplore Node # If "Explore" was requested, run it.
    |   |   └──LoadMap Node     # Load the map if we have our location.
    |   └──UnPlugRobot Node     # Handle robot AC connection status and unplug requests.
    └──ToDo Priority            # A Priority to tick through the "task" branches.
       ├──Premption Priority    # "Emergency" tasks.
       ├──Jobs MemPriority      # User requested normal priority tasks
       └──Idle MemPriority      # Robot initiated tasks for when idle

## Quick Definitions ##

Sequence stops at the first non-success and returns the status,
or returns success if ALL succeed.
Priority runs the next item if one FAILS,
otherwise it returns SUCCESS/RUNNING/ERROR of the first task that gives it.

Full documentation is at https://github.com/renatopp/behavior3js/wiki

## Explanation of Nodes, Sequences and Priorities ##

### Robot Sequence
Root sequence for the entire tree.

#### Poll Node
There are always things that need to be polled. Since the behavior tree operates on a timeOut function that "ticks" periodically,
putting any required polling operations in a node is perfect.

#### Startup Sequence
Tasks that must be done before the robot can function.

##### StartROS Node
Wait for start ROS request = FAILURE
                Starting = RUNNING
                ROS Up and running = SUCCESS

##### Priority
###### AutoExplore Node
If this was requested, run it
                        return RUNNING.
                        else return FAILURE.
###### LoadMap Node
If we have the map name, load it, otherwise perform tasks to find map, such as:
- zbarcam to find a QR Code
- Text me for map name.
- Speak

Once we have a map loaded return SUCCESS.
until then, return FAILURE.
if AutoExplore and LoadMap fail, this node fails.

##### UnPlugRobot Node
When robot is unplugged return SUCCESS,
- If it is unplugING return RUNNING,
- Like "LoadMap", try to get unplugged either by asking others, to unplug it with text/voice,
- or getting permission to unplug via text/voice or QR Code and self unplugging

#### ToDo Priority:
Once "Startup Sequence" gives success, we need to tick everything until something starts RUNNING, ERROR or SUCCES, so insert a Priority here.
##### Preemption Priority
Things that need to take over robot's functions such as emergencies so we check each one.
In theory these are due to emergencies,
I have in mind internal ones, like low battery, unexpected motor shutoffs (fuses), cliffs, etc.
Though it could also be used for outside requests that are emergency level.
Priority Nodes that will all be check, every time, in order.
###### TestPreempt1 Node
Throw in two tests first so we can test this tree.
###### TestPreempt2 Node
Throw in two tests first so we can test this tree.

##### Jobs MemPriority
Once a Job is RUNNING, it will be called repeatedly until it is done, and jobs above it will not be checked.
MemSequence is similar to Sequence node, but when a child returns a RUNNING state, its index is recorded and in the next tick the MemPriority call the child recorded directly, without calling previous children again."
These are jobs given to the robot by outside stimulous, such as user request, or possible automated systems like alarm system, but remember these are not "priorities."
Normal Job Nodes that will be cycled through until one starts, and then it will be ticked to completion.
###### TestJob1 Node
Throw in two tests first so we can test this tree.
###### TestJob2 Node
Throw in two tests first so we can test this tree.

##### Idle MemPriority
Idle behaviors for when there is nothing for the robot to do. These are things the robot will "decide" to do on it's own. Possibly this needs a randomizer, or maybe the "randomizer" will be built into the nodes themselves? (Randomly start vs. fail)
Idle Behavior Nodes that will be cycled through until one starts and then it will be ticked to completion.
###### TestIdle1 Node
Throw in two tests first so we can test this tree.
###### TestIdle2 Node
Throw in two tests first so we can test this tree.

Everything has a priority (top to bottom), so even some jobs preempt others.

I need to build a test node to test the function of my tree.
I can import the same tree in both the test node and the real node.

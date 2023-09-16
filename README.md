# dexhand_gesture_controller
ROS 2 Package for generating gestures on the DexHand using a high level semantic interface

## What is this package used for?
There are a number of potential applications for the gesture controller:

1) **Demonstration of DexHand** - This is a quick way to get a DexHand robot hand to perform a variety of interesting poses. It may be a good entry point into understanding how to pose the hand or do basic operations.
2) **Interface for LLM's or Speech Recognition** - Originally, the package was designed for a demonstration which allows the user to ask a LLM questions, and to use the DexHand gestures as the output device. This is an interesting application.
3) **Foundation for Other Packages** - This is an interesting baseline for creating other high level packages with fixed poses. Feel free to experiment with your own gestures and ideas. 

## How does it work?
It's super simple. The DexHand Gesture Controller subscribes to a single ROS Topic (dexhand_gesture) and accepts String commands which, if recognized in the lookup table of poses, will pose the hand into the appropriate pose. 

There is a simulation version which runs in RVIZ2, and a connection to the physical DexHand via Bluetooth LE.

## Launch Files

To run the package in simulation mode:

`ros2 launch dexhand_gesture_controller simulation.launch.py`

This will open up an RVIZ2 window and the associated nodes to simulate a DexHand. 


To run the package against hardware:

(TBD)

## Sending Gesture Commands

With the simulation and/or physical hardware running (you can run them both at the same time if you wish). You can send messages to the **dexhand_gesture** ROS topic. From the command line, this is done as follows:

`ros2 topic pub -1 /dexhand_gesture std_msgs/msg/String "data: '<pose>'"`

Where *pose* is any of the named hand positions found in the Python script such as fist, peace, horns, point, and so forth. To return to the default hand pose you use *'reset'* as the argument. 

For example, to form a fist:

`ros2 topic pub -1 /dexhand_gesture std_msgs/msg/String "data: 'fist'"`

And then to return to the base pose:

`ros2 topic pub -1 /dexhand_gesture std_msgs/msg/String "data: 'reset'"`





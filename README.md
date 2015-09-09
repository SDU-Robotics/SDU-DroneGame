# SDU-DroneGame

## Requirements
ROS must be installed.

## How to start the drone game
Open three terminals and run the following commands in them.
It is assumed that the terminals initially is in the 
SDU-DroneGame directory.

Terminal 1
    source /opt/ros/indigo/setup.bash
    roscore

Terminal 2
    source /opt/ros/indigo/setup.bash
    cd DroneGame/MarkerTracking/
    python MarkerLocator.py

Terminal 3
    source /opt/ros/indigo/setup.bash
    cd DroneGame/VildeVenner/
    python VildeVenner.py

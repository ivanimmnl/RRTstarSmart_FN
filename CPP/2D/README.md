1. Build the package
2. rosrun drone_waypoint_2d Main
3. Use the generated waypoints.yaml in the waypoint_navigator package by RotorS

waypoint_navigator: https://github.com/ethz-asl/waypoint_navigator

Make sure to change the yaml file in the waypoint_navigator.launch to waypoints.yaml

**If turtlebot is being used, uncomment the control part in Main.cpp
**Turtlebot simulation: refer to https://github.com/Mayavan/RRT-star-path-planning-with-turtlebot

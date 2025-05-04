Quanser_QCar_Competition - UIW

Setup and Execution Instructions

Build the Workspace
colcon build
. install/setup.bash
Running the Application
1. Launch Core System:

In Terminal 1, start the navigation system, sensors, and perception modules:

ros2 launch qcar2_navigation qcar2_navigation_virtual.launch.py use_slam:=False map:=/workspaces/isaac_ros-dev/ros2/src/qcar2_nodes/maps/sdcc/sdcc.yaml

Note: Make sure to use the full absolute path to the map file (sdcc.yaml) so that the map server can correctly load the environment.
In our case (inside the Docker container), the correct path was:
/workspaces/isaac_ros-dev/ros2/src/qcar2_nodes/maps/sdcc/sdcc.yaml
If the map does not load, locate the file under the qcar2_nodes package inside the maps/sdcc folder and adjust the path accordingly.

This launch file includes:
- The virtual QCar2 hardware interface
- RViz2 for visualization
- Nav2 navigation stack (using MPPI controller)
- Lane detection node (PD-based image processing)
- Stop sign and red light detection nodes
- Motor multiplexer node to coordinate control
2. Launch Waypoint Mission:

In Terminal 2, start the waypoint mission script:

ros2 run qcar2_navigation pick_drop_reset.py

This node sends three sequential waypoint paths to the Nav2 stack:
- Drive from Taxi Hub to Pick-Up Zone
- Drive from Pick-Up to Drop-Off
- Return from Drop-Off to Taxi Hub

Video Result Link:
https://youtu.be/cDVJIitHL1E 
The LED color will automatically update after each phase:
- Red before pick-up
- Green after pick-up
- Blue after drop-off
- Red again upon return to the taxi area

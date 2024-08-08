Implement the Obstacle Avoidance on the 4 wheel bot integrating it with GPS based Autonomous traversal.

ROS nodes used:
1)Navigation: publishes bearing and distance to the objective along with the orientation of the rover and a boolean which is true if the rover is in line with the global plan.
2)Obstacle_avoidance: This node detects obstacles and navigates around them, it is an updated version of the movement_controller node used in the previous task. This node uses 'bug 2'.

ROS messages used:
navigation_data: contains 3 float64 values, bearing, distance and orientation and 1 boolean isOnLine

ROS topics used:
/navigation_data: provides bearing, distance, orientation and a boolean which is true if the rover is in line with the global plan, this is used for the bug 2 algorithm
/cmd_vel: commands the skid steer plugin in the rover to make it move.


YouTube Video:https://youtu.be/lUcrUdDcnvk?si=YOBpygot2ixuQmKW

RQT Graph:
![rqt_graph](https://github.com/MRM-AIA-TP-2025/MRM_DillonAsherAlmeida/assets/149039357/3287d06e-1f79-48ef-b88e-b76734559a07)

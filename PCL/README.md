Implement the Obstacle Avoidance on the 4 wheel bot integrating it with GPS based Autonomous traversal.

ROS nodes used:
1)Navigation: publishes bearing and distance to the objective along with the orientation of the rover and a boolean which is true if the rover is in line with the global plan.
2)PCL: This node proccesses point cloud data and publishes obstacle locations to pcl_data, it uses ransac algorithm to remove the gound plane.
2)Obstacle_avoidance_PCL: This node implements the bug 2 algorithm, it subsribes to pcl_data for obstacle detection

ROS messages used:
navigation_data: contains 3 float64 values, bearing, distance and orientation and 1 boolean isOnLine
pcl_data_msg: contains 3 boolean, front, left and right

ROS topics used:
/navigation_data: provides bearing, distance, orientation and a boolean which is true if the rover is in line with the global plan, this is used for the bug 2 algorithm
/pcl_data: provides the location of obstacles, left, right or infront.
/cmd_vel: commands the skid steer plugin in the rover to make it move.


YouTube Video:https://youtu.be/cDl9fq7iY08

RQT Graph:
![rosgraph](https://github.com/MRM-AIA-TP-2025/MRM_DillonAsherAlmeida/assets/149039357/2eaba99b-fd08-4085-ad02-58d510f96849)

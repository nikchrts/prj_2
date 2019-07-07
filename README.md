# Robotics Course - Politecnico di Milano

## Team Members
Fabio Canazza, 920125  
Giacomo Bertollino, 919979  
Nikolaos Chairetis, 916092

## Goals
Compute accurate and high frequency GPS position fusing. We use the following:
- odometry, which is computed from wheel encoders and the steering angle
- IMU data (Piksi Multi board)
- GPS data (Piksi Multi board)

More specifically, we had to:
- Compute the odometry using a node that subscribes to bag data and publishes a nav_msgs/Odometry topic
- Use the *imu_tools* package to elaborate the raw IMU data. A complementary filter was used for this purpose.
- Use of *robot_localization* package to fuse all the available data and get the estimated position at 30 Hz. An Extended Kalman Filter node and a navsat node were used for this purpose.

## Files
For this project, we created 2 files; one cpp file for the odometry node and one launch file for the data fusing and the state estimation.
- **ack_odom.cpp**:
In this node we subscribe to the topic *speedsteer* which contains the steering angle and the velocity speed. Using the Ackerman model, we compute the odometry.
An nav_msgs/Odometry topic, named *navigation/odom*, is published using *odom* as frame_id and *base_link* as child_frame_id. In addition, a tf is published using 
as frame_id *odom* and as child_frame_id *car*. The selection of different child_frame_id names was done for comparison purposes.
- **odom.launch**:
The main file of the project.
  1. Initially, we use the parameter *use_sim_time* in order to be able to use properly the bag data (and the corresponding time). The odometry node is initialized and three static tf are set. Their presence 
preserve the proper function of the estimation and fusing. The most important of them is the one between the frames *map* and *odom*, which is used for the GPS data fusing.
  2. Next, the IMU data are fused using the complementary filter, which gets the bag data and outputs a topic containing the estimated orientation of the car.
This topic together with the output of the odometry node are used as inputs to the EKF node, in order to estimate a car position, with respect to the *odom* frame (local positioning).
  3. Last step, the GPS fusing, which is accomplished with the *navsat* node

<br> During the demonstration, the tf tree should normally be similar with the following image
![tf-tree](https://raw.githubusercontent.com/nikchrts/prj_1/master/tf_tree.png)

## Further Considerations
* The bag should be manually executed in order the demonstration of the project to be meaningful.
* Our interpretation for the reset and the (x,y) position configuration is to only change accordingly the position of the car and all the other variables to remain exactly the same. That means that the car will be moved to the desired position and it will start moving to a different circular trajectory than the initial one, since the rest of the variables (eg yaw angle) do not experience a change.
* The car is moving initially using the Differential Drive Kinematics. 

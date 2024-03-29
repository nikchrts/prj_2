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
  1. Initially, we use the parameter *use_sim_time* in order to be able to use properly the bag data (and the corresponding time). The odometry node is initialized and one static tf (between the frames *map* and *odom*) is set to preserve the proper function of the GPS fusing.
  2. Next, the IMU data are fused using the complementary filter, which gets the bag data and outputs a topic containing the estimated orientation of the car (*imu/data*). This topic together with the output of the odometry node (*navigation/odom*) are used as inputs to the EKF node, in order to estimate a car position, with respect to the *odom* frame (local positioning). Important parameters for the EKF implementation are:
      - *two_d_mode*: the estimation is being made in a 2D plane. Having said that, the important variables are x, y position and the yaw angle (and their corresponding velocities).
      - frame names: the world frame value declares which tf will be used for the estimation. Under this configurations, we will the odom-base_link tf, where the local positioning actually takes place.
      - input configurations: as said above, x, y and yaw values are of interest. We use the odometry topic for the first two (x and y) and the imu orientation for the yaw.
  3. Last step, the GPS fusing, which is accomplished with the *navsat* node. We get the GPS data in long/lat form and we combine it with the other data to get the (global) estimations of x, y values. For the GPS estimation the tf from *map* to *odom* should be used and therefore the static tf has been added in the beginning. The proper offset values are critically important for the performance. These values were found either from lab examples (magnetic declination) or by observing the performance of the output for different values (yaw).

At the end, the results were demonstrated using *mapviz* for both odometry and GPS data. The blue lines represent the final, estimated result, whereas the red lines correspond to raw GPS data (GPS plots) and to the odometry data of the odometry node (odometry plot).

<p align="center">
  <img src="https://github.com/nikchrts/prj_2/blob/master/mapviz-odom.png" alt="drawing" width="780">
  <img src="https://github.com/nikchrts/prj_2/blob/master/mapviz-gps.png" alt="drawing" width="780">
</p>

The tf tree of our implementation is shown below. Notice that from *odom* we have two child frames. The *base_link* frame is the actual being used for the local position estimation, whereas the *car* frame is only used from odometry node.

<p align="center">
  <img src="https://github.com/nikchrts/prj_2/blob/master/frames.png" alt="drawing" width="780">
</p>

## How to use it
To integrate the project in your own workspace and run it properly, follow these simple steps:
- by extracting the contents of the zip file, a 'prj_2' folder should be created
- place it inside the `src` folder of your catkin workspace
- next, cd to catkin workspace and compile the workspace, using `catkin_make`.
- type `roslaunch prj_2 odom.launch` Remember!! The bag should have been also executed.

## Further Considerations
* The bag should be manually executed using the command `rosbag play --clock <path-to-bag-file>`. The *--clock* parameter is essential for the proper performance.

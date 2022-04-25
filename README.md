# als_ros

An Advanced Localization System [1] for the use in Robot Operating System (als_ros) is a localization package with 2D LiDAR. It contains following functions;

- Robust localization based on simultaneous sensor measurement class estimation [2],
- Reliability estimation based on a simple localization correctness classifier [3],
- Misalignment recognition using Markov random fields with fully connected latent variables [4],
- Quick re-localization based on fusion of pose tracking and global localization using the importance sampling [5].



# How to install and use

## How to install

You need to install ROS environment first. Then, type following commands.

```
$ git clone https://github.com/NaokiAkai/als_ros.git
$ cd als_ros
$ catkin_make
$ source devel/setup.bash
```

If you do not want to make a new workspace for als_ros, please copy the als_ros package to your workspace.



## How to use

You need to publish sensor_msgs::LaserScan, nav_msgs::Odometry, and nav_msgs::OccupancyGrid. Default topic names of these messages are "/scan", "/odom", and "/map". 

Also, you need to execute static_transform_publisher for between the base link to the laser sensor. Default frame names of these frames are "base_link" and "laser".

Please refer the ROS documents for setting them.



Then, you can use the localization software as

```
$ roslaunch roslaunch als_ros mcl.launch
```

In default, localization for pose tracking with the robust localization and reliability estimation techniques presented in [2, 3] is executed.



If you want to use estimation of localization failure probability with misalignment recognition, please set use_mrf_failure_detector flag to true.

```
$ roslaunch als_ros mcl.launch use_mrf_failure_detector:=true
```



If you want to use quick fusion of pose tracking and global localization, please set use_gl_pose_sampler flag to true.

```
$ roslaunch als_ros mcl.launch use_gl_pose_sampler:=true
```



## Parameter descriptions

There are launch files in the als_ros package. Descriptions for all the parameters are written in the files.









# References

[1] Naoki Akai．"An advanced localization system: Performance improvement of localization for mobile robots and its implementation," CORONA PUBLISHING CO., LTD (to be appeared, in Japanese).

[2] Naoki Akai, Luis Yoichi Morales, and Hiroshi Murase. "Mobile robot localization considering class of sensor observations," In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3159-3166, 2018.

[3] Naoki Akai, Luis Yoichi Morales, Hiroshi Murase. "Simultaneous pose and reliability estimation using convolutional neural network and Rao-Blackwellized particle filter," *Advanced Robotics*, vol. 32, no. 17, pp. 930-944, 2018.

[4] Naoki Akai, Luis Yoichi Morales, Takatsugu Hirayama, and Hiroshi Murase. "Misalignment recognition using Markov random fields with fully connected latent variables for detecting localization failures," *IEEE Robotics and Automation Letters*, vol. 4, no. 4, pp. 3955-3962, 2019.

[5] Naoki Akai, Takatsugu Hirayama, and Hiroshi Murase. "Hybrid localization using model- and learning-based methods: Fusion of Monte Carlo and E2E localizations via importance sampling," In *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 6469-6475, 2020.


# als_ros

An Advanced Localization System [1] for Robot Operating System use (als_ros) is a localization package with 2D LiDAR. als_ros contains following functions;

- Robust localization based on sensor measurement class estimation [2],
- Reliability estimation based on Bayesian filtering with a simple classifier of localization correctness [3],
- Misalignment recognition using Markov random fields with fully connected latent variables [4],
- Quick re-localization based on fusion of pose tracking and global localization via the importance sampling [5].

These details can be seen at [Reliable Monte Carlo Localization for Mobile Robots (arXiv preprint)](https://arxiv.org/abs/2205.04769).

[Demonstration video](https://www.youtube.com/watch?v=wsoXvUgJvWk) showing comparison of als_ros with ROS amcl.



# How to install and use

## How to install

ROS environment is needed to be installed first. I confirmed that als_ros works on **Ubuntu 18.04** with melodic and **Ubuntu 20.04** with noetic.

als_ros can be installed with following commands.

```
$ git clone https://github.com/NaokiAkai/als_ros.git
$ cd als_ros
$ catkin_make
$ source devel/setup.bash
```

If you do not want to make a new workspace for als_ros, please copy the als_ros package to your workspace. The cloned directory has a ROS workspace.



## How to use

Following messages (topics) are needed to be published; 

- sensor_msgs::LaserScan (/scan)
- nav_msgs::Odometry (/odom)
- nav_msgs::OccupancyGrid (/map)

Names inside of the brackets are default topic names.

Also, static transformation between following two frames is needed to be set.

- origin of a robot (base_link)
- 2D LiDAR (laser)

Names inside of the brackets are default frame names.

There are launch files in the als_ros package. These names can be changed in **mcl.launch**.



After setting the topics and transformation, the localization software can be used with mcl.launch.

```
$ roslaunch als_ros mcl.launch
```

In default, localization for pose tracking with the robust localization and reliability estimation techniques presented in [2, 3] is executed.



If you want to use fusion of pose tracking and global localization, please set use_gl_pose_sampler flag to true.

```
$ roslaunch als_ros mcl.launch use_gl_pose_sampler:=true
```

In als_ros, global localization is implemented using the free-space feature presented in [6].



If you want to use estimation of localization failure probability with misalignment recognition, please set use_mrf_failure_detector flag to true.

```
$ roslaunch als_ros mcl.launch use_mrf_failure_detector:=true
```



## Parameter descriptions

Descriptions for all the parameters are written in the launch files. I am planning to make a more precise document.



# Citation

[arXiv preprint](https://arxiv.org/abs/2205.04769) is available. If you used als_ros in your research, please cite the preprint.

```
Naoki Akai. "Reliable Monte Carlo Localization for Mobile Robots," arXiv:2205.04769, 2022.
```



```
@article{Akai2022arXiv:ReliableMC,
    title = {Reliable Monte Carlo Localization for Mobile Robots},
    author = {Akai, Naoki},
    journal = {arXiv:2205.04769},
    year = {2022}
}
```



# References

[1] Naoki Akai．"An advanced localization system using LiDAR: Performance improvement of localization for mobile robots and its implementation," CORONA PUBLISHING CO., LTD (to be appeared, in Japanese).

[2] Naoki Akai, Luis Yoichi Morales, and Hiroshi Murase. "Mobile robot localization considering class of sensor observations," In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3159-3166, 2018.

[3] Naoki Akai, Luis Yoichi Morales, Hiroshi Murase. "Simultaneous pose and reliability estimation using convolutional neural network and Rao-Blackwellized particle filter," *Advanced Robotics*, vol. 32, no. 17, pp. 930-944, 2018.

[4] Naoki Akai, Luis Yoichi Morales, Takatsugu Hirayama, and Hiroshi Murase. "Misalignment recognition using Markov random fields with fully connected latent variables for detecting localization failures," *IEEE Robotics and Automation Letters*, vol. 4, no. 4, pp. 3955-3962, 2019.

[5] Naoki Akai, Takatsugu Hirayama, and Hiroshi Murase. "Hybrid localization using model- and learning-based methods: Fusion of Monte Carlo and E2E localizations via importance sampling," In *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 6469-6475, 2020.

[6] Alexander Millane, Helen, Oleynikova, Juan Nieto, Roland Siegwart, and César Cadena. "Free-space features: Global localization in 2D laser SLAM using distance function maps," In *Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 1271-1277, 2019.



# License

Mozilla Public License Version 2.0


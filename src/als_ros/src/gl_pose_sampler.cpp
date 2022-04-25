/****************************************************************************
 * ALSROS: Advanced 2D Localization Systems for ROS use
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <als_ros/GLPoseSampler.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gl_pose_sampler");
    als_ros::GLPoseSampler sampler;
    sampler.spin();
    return 0;
}
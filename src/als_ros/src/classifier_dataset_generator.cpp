/****************************************************************************
 * als_ros: An Advanced Localization System for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <als_ros/ClassifierDatasetGenerator.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "classifier_dataset_generator");
    als_ros::ClassifierDatasetGenerator generator;
    generator.datasetGenerationInit();
    generator.generateDataset();
    return 0;
}

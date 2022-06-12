/****************************************************************************
 * als_ros: An Advanced Localization System for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * Licensed under the Apache License, Version 2.0 (the “License”);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an “AS IS” BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <als_ros/Pose.h>
#include <als_ros/ClassifierDatasetGenerator.h>
#include <als_ros/MAEClassifier.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mae_classifier_learning");
    ros::NodeHandle nh("~");

    std::vector<std::string> trainDirs, testDirs;
    std::string classifierDir;
    double maxResidualError, maeHistogramBinWidth;

    nh.param("train_dirs", trainDirs, trainDirs);
    nh.param("test_dirs", testDirs, testDirs);
    nh.param("classifier_dir", classifierDir, classifierDir);
    nh.param("max_residual_error", maxResidualError, maxResidualError);
    nh.param("histogram_bin_width", maeHistogramBinWidth, maeHistogramBinWidth);

    std::vector<als_ros::Pose> gtPosesTrain, successPosesTrain, failurePosesTrain;
    std::vector<sensor_msgs::LaserScan> scansTrain;
    std::vector<std::vector<double>> successResidualErrorsTrain, failureResidualErrorsTrain;

    std::vector<als_ros::Pose> gtPosesTest, successPosesTest, failurePosesTest;
    std::vector<sensor_msgs::LaserScan> scansTest;
    std::vector<std::vector<double>> successResidualErrorsTest, failureResidualErrorsTest;

    als_ros::ClassifierDatasetGenerator generator;
    generator.setTrainDirs(trainDirs);
    generator.setTestDirs(testDirs);
    generator.readTrainDataset(gtPosesTrain, successPosesTrain, failurePosesTrain, scansTrain, successResidualErrorsTrain, failureResidualErrorsTrain);
    generator.readTestDataset(gtPosesTest, successPosesTest, failurePosesTest, scansTest, successResidualErrorsTest, failureResidualErrorsTest);

    als_ros::MAEClassifier classifier;
    classifier.setClassifierDir(classifierDir);
    classifier.setMaxResidualError(maxResidualError);
    classifier.setMAEHistogramBinWidth(maeHistogramBinWidth);
    classifier.learnThreshold(successResidualErrorsTrain, failureResidualErrorsTrain);
    classifier.writeClassifierParams(successResidualErrorsTest, failureResidualErrorsTest);
    classifier.writeDecisionLikelihoods();

    return 0;
}

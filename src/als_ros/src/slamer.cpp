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
#include <als_ros/SLAMER.h>

int main(int argc, char **argv) {
    if (argv[1] == NULL) {
        ROS_ERROR("argv[1] must be a yaml file for a indoor semantic map (ism).");
        exit(1);
    }

    ros::init(argc, argv, "slamer");
    als_ros::SLAMER slamer(argv[1]);
    double localizationHz = slamer.getLocalizationHz();
    ros::Rate loopRate(localizationHz);

    while (ros::ok()) {
        ros::spinOnce();
        slamer.updateParticlesByMotionModel();
        slamer.setCanUpdateScan(false);
        slamer.calculateLikelihoodsByMeasurementModel();
        slamer.calculateLikelihoodsBySLAMER();
        slamer.recognizeObjectsWithMapAssist();
        slamer.calculateLikelihoodsByDecisionModel();
        slamer.calculateGLSampledPosesLikelihood();
        slamer.calculateAMCLRandomParticlesRate();
        slamer.calculateEffectiveSampleSize();
        slamer.estimatePose();
        slamer.resampleParticles();
        slamer.publishROSMessages();
        slamer.publishSLAMERROSMessages();
        slamer.broadcastTF();
        // slamer.plotLikelihoodMap();
        slamer.setCanUpdateScan(true);
        slamer.printResult();
        loopRate.sleep();
    }

    return 0;
}
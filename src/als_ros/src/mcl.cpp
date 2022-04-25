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
#include <als_ros/MCL.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "mcl");

    als_ros::MCL mcl;
    double localizationHz = mcl.getLocalizationHz();
    ros::Rate loopRate(localizationHz);

    while (ros::ok()) {
        ros::spinOnce();
        mcl.updateParticlesByMotionModel();
        mcl.setCanUpdateScan(false);
        mcl.calculateLikelihoodsByMeasurementModel();
        mcl.calculateLikelihoodsByDecisionModel();
        mcl.calculateGLSampledPosesLikelihood();
        mcl.calculateAMCLRandomParticlesRate();
        mcl.calculateEffectiveSampleSize();
        mcl.estimatePose();
        mcl.resampleParticles();
        mcl.publishROSMessages();
        mcl.broadcastTF();
        // mcl.plotLikelihoodMap();
        mcl.setCanUpdateScan(true);
        mcl.printResult();
        loopRate.sleep();
    }

    return 0;
}

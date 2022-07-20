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

#ifndef __SLAMER_H__
#define __SLAMER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <als_ros/MCL.h>
#include <als_ros/ISM.h>
#include <als_ros/LineObjectRecognition.h>

namespace als_ros {

class SLAMER : public MCL {
    ros::NodeHandle nh_;
    ros::Publisher ismPointsPub_, coloredScanPointsPub_, lineObjectsPub_, spatialLineObjectsPub_;
    std::string ismPointsName_, coloredScanPointsName_, lineObjectsName_, spatialLineObjectsName_;
    bool publishISMPoints_, publishColoredScanPoints_;

    ISM ism_;
    std::vector<std::string> objectNames_;
    double mapResolution_;

    double normConstHit_, denomHit_, zHit_, measurementModelRandom_;

    sensor_msgs::LaserScan scan_;
    LineObjectRecognition recog_;
    std::vector<LineObject> lineObjects_, spatialLineObjects_;
    std::vector<std::vector<double>> lineObjectsProbs_, spatialLineObjectsProbs_;

public:
    SLAMER(std::string ismYamlFile):
        nh_("~"),
        ismPointsName_("/ism_points"),
        coloredScanPointsName_("/slamer_colored_scan_points"),
        lineObjectsName_("/slamer_line_objects"),
        spatialLineObjectsName_("/slamer_spatial_line_objects"),
        publishISMPoints_(true),
        publishColoredScanPoints_(true)
    {
        // slamer parameters
        nh_.param("ism_points_name", ismPointsName_, ismPointsName_);
        nh_.param("colored_scan_points_name", coloredScanPointsName_, coloredScanPointsName_);
        nh_.param("line_objects_name", lineObjectsName_, lineObjectsName_);
        nh_.param("spatial_line_objects_name", spatialLineObjectsName_, spatialLineObjectsName_);
        nh_.param("publish_ism_points", publishISMPoints_, publishISMPoints_);
        nh_.param("publish_colored_scan_points", publishColoredScanPoints_, publishColoredScanPoints_);

        // scan feature parameters
        double scanRangeMax, scanMapReso, scanOrientationHistReso, minScanOrientationProb;
        double minSpatialLineObjectLength, maxSpatialLineObjectLength, mergePointsDist;
        nh_.param("scan_range_max", scanRangeMax, 20.0);
        nh_.param("scan_map_reso", scanMapReso, 0.05);
        nh_.param("scan_orientation_hist_reso", scanOrientationHistReso, 5.0);
        nh_.param("min_scan_orientation_prob", minScanOrientationProb, 0.1);
        nh_.param("min_spatial_line_object_length", minSpatialLineObjectLength, 0.5);
        nh_.param("max_spatial_line_object_length", maxSpatialLineObjectLength, 2.5);
        nh_.param("merge_points_dist", mergePointsDist, 0.5);
        scanOrientationHistReso *= M_PI / 180.0;

        // publisher
        if (publishISMPoints_)
            ismPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(ismPointsName_, 1, this);
        if (publishColoredScanPoints_)
            coloredScanPointsPub_ = nh_.advertise<sensor_msgs::PointCloud2>(coloredScanPointsName_, 1);
        lineObjectsPub_ = nh_.advertise<visualization_msgs::Marker>(lineObjectsName_, 1);
        spatialLineObjectsPub_ = nh_.advertise<visualization_msgs::Marker>(spatialLineObjectsName_, 1);

        // ism
        ism_.readISM(ismYamlFile);
        objectNames_ = ism_.getObjectNames();
        mapResolution_ = ism_.getMapResolution();
        if (publishISMPoints_) {
            sensor_msgs::PointCloud2 ismPointsMsg = ism_.getISMPointsAsPC2();
            ismPointsMsg.header.frame_id = getMapFrame();
            ismPointsMsg.header.stamp = ros::Time::now();
            ismPointsPub_.publish(ismPointsMsg);
        }

        // slamer parameters used in mcl
        normConstHit_ = getNormConstHit();
        denomHit_ = getDenomHit();
        zHit_ = getZHit();
        measurementModelRandom_ = getMeasurementModelRandom();

        // line object recognizer
        recog_.init(scanRangeMax, scanMapReso, scanOrientationHistReso, minScanOrientationProb, minSpatialLineObjectLength, maxSpatialLineObjectLength, mergePointsDist);
    }

    void calculateLikelihoodsBySLAMER(void) {
        scan_ = getScan();
        recog_.recognizeLineObjects(scan_, lineObjects_, lineObjectsProbs_, spatialLineObjects_, spatialLineObjectsProbs_);
        if ((int)lineObjects_.size() == 0) {
            ROS_INFO("Likelihood calculation with SLAMER is ignored since actual line objects were not detected.");
            return;
        }

        int particlesNum = getParticlesNum();
        setMCLPoseStamp(scan_.header.stamp);
        Pose baseLink2Laser = getBaseLink2Laser();
        double xo = baseLink2Laser.getX();
        double yo = baseLink2Laser.getY();
        double yawo = baseLink2Laser.getYaw();
        std::vector<Pose> sensorPoses(particlesNum);
        for (int i = 0; i < particlesNum; ++i) {
            Pose particlePose = getParticlePose(i);
            double yaw = particlePose.getYaw();
            double sensorX = xo * cos(yaw) - yo * sin(yaw) + particlePose.getX();
            double sensorY = xo * sin(yaw) + yo * cos(yaw) + particlePose.getY();
            double sensorYaw = yawo + yaw;
            Pose sensorPose(sensorX, sensorY, sensorYaw);
            sensorPoses[i] = sensorPose;
            double w = log(getParticleW(i));
            setParticleW(i, w);
        }

        // clearLikelihoodShiftedSteps();
        for (int i = 0; i < (int)lineObjects_.size(); ++i) {
            double max;
            for (int j = 0; j < particlesNum; ++j) {
                std::vector<double> lineObjectPrior = getLineObjectPrior(sensorPoses[j], lineObjects_[i]);
                double pCloseDoor = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::CLOSE_DOOR) * lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR];
                double pCloseGlassDoor = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::CLOSE_GLASS_DOOR) * lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR];
                double pFence = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::FENCE) * lineObjectPrior[(int)LineObjectLabel::FENCE];
                double pOthers = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::OTHERS) * lineObjectPrior[(int)LineObjectLabel::OTHERS];
                double p = pCloseDoor + pCloseGlassDoor + pFence + pOthers;
                if (p < 10.0e-9)
                    p = 10.0e-9;
                if (p > 1.0 - 10.0e-9)
                    p = 1.0 - 10.0e-9;

                double w = getParticleW(j);
                w += log(p);
                setParticleW(j, w);
                if (j == 0) {
                    max = w;
                } else {
                    if (max < w)
                        max = w;
                }
            }

            if (max < -300.0) {
                for (int j = 0; j < particlesNum; ++j) {
                    double w = getParticleW(j) + 300.0;
                    setParticleW(j, w);
                }
//                addLkelihoodShiftedSteps(true);
            } else {
//                addLikelihoodShiftedSteps(false);
            }
        }

        double sum = 0.0;
        double max;
        int maxIdx;
        for (int i = 0; i < particlesNum; ++i) {
            double w = exp(getParticleW(i));
            setParticleW(i, w);
            sum += w;
            if (i == 0) {
                max = w;
                maxIdx = i;
            } else if (max < w) {
                max = w;
                maxIdx = i;
            }
        }
        setTotalLikelihood(sum);
        setAverageLikelihood(sum / (double)particlesNum);
        setMaxLikelihood(max);
        setMaxLikelihoodParticleIdx(maxIdx);
    }

    void recognizeObjectsWithMapAssist(void) {
        int maxLikelihoodParticleIdx = getMaxLikelihoodParticleIdx();
        Pose mlPose = getParticlePose(maxLikelihoodParticleIdx);
        Pose baseLink2Laser = getBaseLink2Laser();
        double yaw = mlPose.getYaw();
        double sensorX = baseLink2Laser.getX() * cos(yaw) - baseLink2Laser.getY() * sin(yaw) + mlPose.getX();
        double sensorY = baseLink2Laser.getX() * sin(yaw) + baseLink2Laser.getY() * cos(yaw) + mlPose.getY();
        double sensorYaw = baseLink2Laser.getYaw() + yaw;
        Pose sensorPose(sensorX, sensorY, sensorYaw);

        for (int i = 0; i < (int)lineObjects_.size(); ++i) {
            std::vector<double> lineObjectPrior = getLineObjectPrior(sensorPose, lineObjects_[i]);
            double pCloseDoor = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::CLOSE_DOOR) * lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR];
            double pCloseGlassDoor = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::CLOSE_GLASS_DOOR) * lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR];
            double pFence = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::FENCE) * lineObjectPrior[(int)LineObjectLabel::FENCE];
            double pOthers = calculateLineObjectClassificationModel(lineObjectsProbs_[i], (int)LineObjectLabel::OTHERS) * lineObjectPrior[(int)LineObjectLabel::OTHERS];

            double sum = pCloseDoor + pCloseGlassDoor + pFence + pOthers;
            lineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_DOOR] = pCloseDoor / sum;
            lineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_GLASS_DOOR] = pCloseGlassDoor / sum;
            lineObjectsProbs_[i][(int)LineObjectLabel::FENCE] = pFence / sum;
            lineObjectsProbs_[i][(int)LineObjectLabel::OTHERS] = pOthers / sum;
        }

        for (int i = 0; i < (int)spatialLineObjects_.size(); ++i) {
            std::vector<double> spatialLineObjectPrior = getSpatialLineObjectPrior(sensorPose, spatialLineObjects_[i]);
            double pOpenDoor = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::OPEN_DOOR) * spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR];
            double pOpenGlassDoor = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::OPEN_GLASS_DOOR) * spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR];
            double pCloseGlassDoor = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::CLOSE_GLASS_DOOR) * spatialLineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR];
            double pFence = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::FENCE) * spatialLineObjectPrior[(int)LineObjectLabel::FENCE];
            double pNoEntryLine = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::NO_ENTRY) * spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY];
            double pFreeSpace = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::FREE_SPACE) * spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE];
            double pOthers = calculateSpatialLineObjectClassificationModel(spatialLineObjectsProbs_[i], (int)LineObjectLabel::OTHERS) * spatialLineObjectPrior[(int)LineObjectLabel::OTHERS];

            double sum = pOpenDoor + pOpenGlassDoor + pCloseGlassDoor + pFence + pNoEntryLine + pFreeSpace + pOthers;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::OPEN_DOOR] = pOpenDoor / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::OPEN_GLASS_DOOR] = pOpenGlassDoor / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_GLASS_DOOR] = pCloseGlassDoor / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::FENCE] = pFence / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::NO_ENTRY] = pNoEntryLine / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::FREE_SPACE] = pFreeSpace / sum;
            spatialLineObjectsProbs_[i][(int)LineObjectLabel::OTHERS] = pOthers / sum;
        }
    }

    void publishSLAMERROSMessages(void) {
        if (publishColoredScanPoints_) {
            sensor_msgs::PointCloud2 coloredScanImgPoints = recog_.getColoredScanImgPoints(lineObjects_, lineObjectsProbs_);
            coloredScanImgPoints.header = scan_.header;
            coloredScanPointsPub_.publish(coloredScanImgPoints);
        }

        visualization_msgs::Marker lineObjects, spatialLineObjects;
        lineObjects.header = scan_.header;
        lineObjects.ns = "slamer_line_objects";
        lineObjects.lifetime = ros::Duration();
        lineObjects.frame_locked = true;
        lineObjects.type = visualization_msgs::Marker::LINE_LIST;
        lineObjects.action = visualization_msgs::Marker::ADD;
        lineObjects.scale.x = 0.2;
        lineObjects.scale.y = 0.0;
        lineObjects.scale.z = 0.0;
        lineObjects.pose.orientation.x = 0.0;
        lineObjects.pose.orientation.y = 0.0;
        lineObjects.pose.orientation.z = 0.0;
        lineObjects.pose.orientation.w = 1.0;

        spatialLineObjects = lineObjects;
        lineObjects.id = 0;
        spatialLineObjects.id = 1;

        for (int i = 0; i < (int)lineObjects_.size(); ++i) {
            double pCloseDoor = lineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_DOOR];
            double pCloseGlassDoor = lineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_GLASS_DOOR];
            double pFence = lineObjectsProbs_[i][(int)LineObjectLabel::FENCE];
            double pOthers = lineObjectsProbs_[i][(int)LineObjectLabel::OTHERS];
            // printf("%d: %lf, %lf, %lf, %lf\n", i, pCloseDoor, pCloseGlassDoor, pFence, pOthers);

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            if (pOthers > pCloseDoor && pOthers > pCloseGlassDoor && pOthers > pFence) {
                // recog_.getObjectColor((int)LineObjectLabel::OTHERS, color.r, color.g, color.b);
                continue;
            } else if (pCloseDoor < 0.5 && pCloseGlassDoor < 0.5 && pFence < 0.5) {
                continue;
            } else if (pCloseDoor > pFence && pCloseDoor > pCloseGlassDoor) {
                recog_.getObjectColor((int)LineObjectLabel::CLOSE_DOOR, color.r, color.g, color.b);
            } else if (pCloseGlassDoor > pFence) {
                recog_.getObjectColor((int)LineObjectLabel::CLOSE_GLASS_DOOR, color.r, color.g, color.b);
            } else {
                recog_.getObjectColor((int)LineObjectLabel::FENCE, color.r, color.g, color.b);
            }

            als_ros::Point lp1 = lineObjects_[i].getP1();
            als_ros::Point lp2 = lineObjects_[i].getP2();
            geometry_msgs::Point p1, p2;
            p1.x = lp1.getX();
            p1.y = lp1.getY();
            p2.x = lp2.getX();
            p2.y = lp2.getY();
            p1.z = p2.z = 0.0;
            lineObjects.points.push_back(p1);
            lineObjects.points.push_back(p2);
            lineObjects.colors.push_back(color);
            lineObjects.colors.push_back(color);
        }

        for (int i = 0; i < (int)spatialLineObjects_.size(); ++i) {
            double pOpenDoor = spatialLineObjectsProbs_[i][(int)LineObjectLabel::OPEN_DOOR];
            double pOpenGlassDoor = spatialLineObjectsProbs_[i][(int)LineObjectLabel::OPEN_GLASS_DOOR];
            double pCloseGlassDoor = spatialLineObjectsProbs_[i][(int)LineObjectLabel::CLOSE_GLASS_DOOR];
            double pFence = spatialLineObjectsProbs_[i][(int)LineObjectLabel::FENCE];
            double pNoEntryLine = spatialLineObjectsProbs_[i][(int)LineObjectLabel::NO_ENTRY];
            double pFreeSpace = spatialLineObjectsProbs_[i][(int)LineObjectLabel::FREE_SPACE];
            double pOthers = spatialLineObjectsProbs_[i][(int)LineObjectLabel::OTHERS];
            // printf("%d: %lf, %lf, %lf, %lf, %lf, %lf %lf\n", i, pOpenDoor, pOpenGlassDoor, pFence, pCloseGlassDoor, pNoEntryLine, pFreeSpace, pOthers);

            std_msgs::ColorRGBA color;
            color.a = 0.5;
            if (pFreeSpace > pOpenDoor && pFreeSpace > pOpenGlassDoor && pFreeSpace > pCloseGlassDoor && pFreeSpace > pFence && pFreeSpace > pNoEntryLine && pFreeSpace > pOthers) {
                // recog_.getObjectColor((int)LineObjectLabel::FREE_SPACE, color.r, color.g, color.b);
                continue;
            } else if (pOthers > pOpenDoor && pOthers > pOpenGlassDoor && pOthers > pCloseGlassDoor && pOthers > pFence && pOthers > pNoEntryLine) {
                // recog_.getObjectColor((int)LineObjectLabel::OTHERS, color.r, color.g, color.b);
                continue;
            } else if (pOpenDoor < 0.5 && pOpenGlassDoor < 0.5 && pCloseGlassDoor < 0.5 && pFence < 0.5 && pNoEntryLine < 0.5) {
                continue;
            } else if (pOpenDoor > pOpenGlassDoor && pOpenDoor > pCloseGlassDoor && pOpenDoor > pFence && pOpenDoor > pNoEntryLine) {
                recog_.getObjectColor((int)LineObjectLabel::OPEN_DOOR, color.r, color.g, color.b);
            } else if (pOpenGlassDoor > pCloseGlassDoor && pOpenGlassDoor > pFence && pOpenGlassDoor > pNoEntryLine) {
                recog_.getObjectColor((int)LineObjectLabel::OPEN_GLASS_DOOR, color.r, color.g, color.b);
            } else if (pCloseGlassDoor > pFence && pCloseGlassDoor > pNoEntryLine) {
                recog_.getObjectColor((int)LineObjectLabel::CLOSE_GLASS_DOOR, color.r, color.g, color.b);
            } else if (pFence > pNoEntryLine) {
                recog_.getObjectColor((int)LineObjectLabel::FENCE, color.r, color.g, color.b);
            } else {
                recog_.getObjectColor((int)LineObjectLabel::NO_ENTRY, color.r, color.g, color.b);
            }

            als_ros::Point lp1 = spatialLineObjects_[i].getP1();
            als_ros::Point lp2 = spatialLineObjects_[i].getP2();
            geometry_msgs::Point p1, p2;
            p1.x = lp1.getX();
            p1.y = lp1.getY();
            p2.x = lp2.getX();
            p2.y = lp2.getY();
            p1.z = p2.z = 0.0;
            spatialLineObjects.points.push_back(p1);
            spatialLineObjects.points.push_back(p2);
            spatialLineObjects.colors.push_back(color);
            spatialLineObjects.colors.push_back(color);
        }

        lineObjectsPub_.publish(lineObjects);
        spatialLineObjectsPub_.publish(spatialLineObjects);
    }

private:
    inline double clipProb(double p) {
        if (p < 10.0e-15)
            return 10.0e-15;
        else if (p > 1.0 - 10.0e-15)
            return 1.0 - 10.0e-15;
        return p;
    }

    inline double calculateDirichletDistribution(std::vector<double> probs, std::vector<double> coefs) {
        double prod1 = 1.0, prod2 = 1.0, sum = 0.0;
        for (int i = 0; i < (int)probs.size(); ++i) {
            prod1 *= std::tgamma(coefs[i]);
            sum += coefs[i];
            prod2 *= pow(probs[i], coefs[i] - 1.0);
        }
        double beta = prod1 / std::tgamma(sum);
        double p = prod2 / beta;
        return clipProb(p);
    }

    double calculateLineObjectClassificationModel(std::vector<double> classificationProbs, int objectID) {
        std::vector<double> probs(4), coefs(4);
        probs[0] = classificationProbs[(int)LineObjectLabel::CLOSE_DOOR];
        probs[0] = classificationProbs[(int)LineObjectLabel::CLOSE_GLASS_DOOR];
        probs[1] = classificationProbs[(int)LineObjectLabel::FENCE];
        probs[2] = classificationProbs[(int)LineObjectLabel::OTHERS];

        double coef1 = 2.0, coef2 = 2.0;
        if (objectID == (int)LineObjectLabel::CLOSE_DOOR)
            coefs[0] = coef1, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef2;
        if (objectID == (int)LineObjectLabel::CLOSE_GLASS_DOOR)
            coefs[0] = coef2, coefs[1] = coef1, coefs[2] = coef2, coefs[3] = coef2;
        else if (objectID == (int)LineObjectLabel::FENCE)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef1, coefs[3] = coef2;
        else
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef1;

        return calculateDirichletDistribution(probs, coefs);
    }

    double calculateSpatialLineObjectClassificationModel(std::vector<double> classificationProbs, int objectID) {
        std::vector<double> probs(7), coefs(7);
        probs[0] = classificationProbs[(int)LineObjectLabel::OPEN_DOOR];
        probs[1] = classificationProbs[(int)LineObjectLabel::OPEN_GLASS_DOOR];
        probs[2] = classificationProbs[(int)LineObjectLabel::CLOSE_GLASS_DOOR];
        probs[3] = classificationProbs[(int)LineObjectLabel::FENCE];
        probs[4] = classificationProbs[(int)LineObjectLabel::NO_ENTRY];
        probs[5] = classificationProbs[(int)LineObjectLabel::FREE_SPACE];
        probs[6] = classificationProbs[(int)LineObjectLabel::OTHERS];

        double coef1 = 2.0, coef2 = 2.0;
        if (objectID == (int)LineObjectLabel::OPEN_DOOR)
            coefs[0] = coef1, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef2, coefs[4] = coef2, coefs[5] = coef2, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::OPEN_GLASS_DOOR)
            coefs[0] = coef2, coefs[1] = coef1, coefs[2] = coef2, coefs[3] = coef2, coefs[4] = coef2, coefs[5] = coef2, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::CLOSE_GLASS_DOOR)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef1, coefs[3] = coef2, coefs[4] = coef2, coefs[5] = coef2, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::FENCE)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef1, coefs[4] = coef2, coefs[5] = coef2, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::NO_ENTRY)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef2, coefs[4] = coef1, coefs[5] = coef2, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::FREE_SPACE)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef2, coefs[4] = coef2, coefs[5] = coef1, coefs[6] = coef2;
        else if (objectID == (int)LineObjectLabel::OTHERS)
            coefs[0] = coef2, coefs[1] = coef2, coefs[2] = coef2, coefs[3] = coef2, coefs[4] = coef2, coefs[5] = coef2, coefs[6] = coef1;

        return calculateDirichletDistribution(probs, coefs);
    }

    double calculateLikelihoodFieldModel(double dist) {
        double p = measurementModelRandom_;
        if (dist >= 0.0) {
            double pHit = normConstHit_ * exp(-(dist * dist) * denomHit_) * mapResolution_;
            p += zHit_ * pHit;
        }
        if (p > 1.0)
            p = 1.0;
        return p;
    }

    std::vector<double> getLineObjectPrior(Pose sensorPose, LineObject lineObject) {
        Point p1 = lineObject.getP1();
        Point p2 = lineObject.getP2();
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        double r1 = sqrt(x1 * x1 + y1 * y1);
        double r2 = sqrt(x2 * x2 + y2 * y2);
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dl = sqrt(dx * dx + dy * dy);
        double yaw1 = sensorPose.getYaw() + atan2(y1, x1);
        double yaw2 = sensorPose.getYaw() + atan2(y2, x2);
        double xx1 = r1 * cos(yaw1) + sensorPose.getX();
        double yy1 = r1 * sin(yaw1) + sensorPose.getY();
        double xx2 = r2 * cos(yaw2) + sensorPose.getX();
        double yy2 = r2 * sin(yaw2) + sensorPose.getY();
        double t = atan2(yy2 - yy1, xx2 - xx1);
        double ddx = mapResolution_ * cos(t);
        double ddy = mapResolution_ * sin(t);

        std::vector<double> lineObjectPrior((int)LineObjectLabel::OTHERS + 1);
        lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR] = 1.0;
        lineObjectPrior[(int)LineObjectLabel::OPEN_DOOR] = 0.0;
        lineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] = 0.0;
        lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] = 1.0;
        lineObjectPrior[(int)LineObjectLabel::FENCE] = 1.0;
        lineObjectPrior[(int)LineObjectLabel::NO_ENTRY] = 0.0;
        lineObjectPrior[(int)LineObjectLabel::FREE_SPACE] = 0.0;
        lineObjectPrior[(int)LineObjectLabel::OTHERS] = 1.0;

        for (double l = 0.0; l <= dl; l += mapResolution_) {
            float doorDist = ism_.getDistance(xx1, yy1, "door");
            float glassDoorDist = ism_.getDistance(xx1, yy1, "glass_door");
            float fenceDist = ism_.getDistance(xx1, yy1, "fence");
            float othersDist = ism_.getDistance(xx1, yy1, "others");
            if ((doorDist < 0.0f && glassDoorDist < 0.0f && fenceDist < 0.0f) || othersDist < 0.0f) {
                lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR] = 0.0;
                lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] = 0.0;
                lineObjectPrior[(int)LineObjectLabel::FENCE] = 0.0;
                lineObjectPrior[(int)LineObjectLabel::OTHERS] = 1.0;
                break;
            } else {
                double pCloseDoor = calculateLikelihoodFieldModel((double)doorDist);
                double pCloseGlassDoor = calculateLikelihoodFieldModel((double)glassDoorDist);
                double pFence = calculateLikelihoodFieldModel((double)fenceDist);
                double pOthers = calculateLikelihoodFieldModel((double)othersDist);

                lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR] *= pCloseDoor;
                lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] *= pCloseGlassDoor;
                lineObjectPrior[(int)LineObjectLabel::FENCE] *= pFence;
                lineObjectPrior[(int)LineObjectLabel::OTHERS] *= pOthers;

                lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR] = clipProb(lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR]);
                lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] = clipProb(lineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR]);
                lineObjectPrior[(int)LineObjectLabel::FENCE] = clipProb(lineObjectPrior[(int)LineObjectLabel::FENCE]);
                lineObjectPrior[(int)LineObjectLabel::OTHERS] = clipProb(lineObjectPrior[(int)LineObjectLabel::OTHERS]);

                recog_.normalizeProb(lineObjectPrior);
            }
            xx1 += ddx;
            yy1 += ddy;
        }
        // printf("%lf, %lf, %lf\n", lineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR], 
        //     lineObjectPrior[(int)LineObjectLabel::FENCE], lineObjectPrior[(int)LineObjectLabel::OTHERS]);
        // printf("\n");
        return lineObjectPrior;
    }

    std::vector<double> getSpatialLineObjectPrior(Pose sensorPose, LineObject spatialLineObject) {
        Point p1 = spatialLineObject.getP1();
        Point p2 = spatialLineObject.getP2();
        double x1 = p1.getX();
        double y1 = p1.getY();
        double x2 = p2.getX();
        double y2 = p2.getY();
        double r1 = sqrt(x1 * x1 + y1 * y1);
        double r2 = sqrt(x2 * x2 + y2 * y2);
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dl = sqrt(dx * dx + dy * dy);
        double yaw1 = sensorPose.getYaw() + atan2(y1, x1);
        double yaw2 = sensorPose.getYaw() + atan2(y2, x2);
        double xx1 = r1 * cos(yaw1) + sensorPose.getX();
        double yy1 = r1 * sin(yaw1) + sensorPose.getY();
        double xx2 = r2 * cos(yaw2) + sensorPose.getX();
        double yy2 = r2 * sin(yaw2) + sensorPose.getY();
        double t = atan2(yy2 - yy1, xx2 - xx1);
        double ddx = mapResolution_ * cos(t);
        double ddy = mapResolution_ * sin(t);

        std::vector<double> spatialLineObjectPrior((int)LineObjectLabel::OTHERS + 1);
        spatialLineObjectPrior[(int)LineObjectLabel::CLOSE_DOOR] = 0.0;
        spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::FENCE] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE] = 1.0;
        spatialLineObjectPrior[(int)LineObjectLabel::OTHERS] = 1.0;
        for (double l = 0.0; l <= dl; l += mapResolution_) {
            float doorDist = ism_.getDistance(xx1, yy1, "door");
            float glassDoorDist = ism_.getDistance(xx1, yy1, "glass_door");
            float fenceDist = ism_.getDistance(xx1, yy1, "fence");
            float doNotEnterLineDist = ism_.getDistance(xx1, yy1, "do_not_enter_line");
            float othersDist = ism_.getDistance(xx1, yy1, "others");
            if (doorDist < 0.0f && glassDoorDist < 0.0f && fenceDist < 0.0 && doNotEnterLineDist < 0.0 && othersDist < 0.0) {
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR] = 0.0;
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] = 0.0;
                spatialLineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] = 0.0;
                spatialLineObjectPrior[(int)LineObjectLabel::FENCE] = 0.0;
                spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY] = 0.0;
                spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE] = 1.0;
                spatialLineObjectPrior[(int)LineObjectLabel::OTHERS] = 0.0;
                break;
            } else {
                double pDoor = calculateLikelihoodFieldModel((double)doorDist);
                double pGlassDoor = calculateLikelihoodFieldModel((double)glassDoorDist);
                double pFence = calculateLikelihoodFieldModel((double)fenceDist);
                double pNoEntryLine = calculateLikelihoodFieldModel((double)doNotEnterLineDist);
                double pOthers = calculateLikelihoodFieldModel((double)othersDist);
                double pFreeSpace;
                if (pDoor > pGlassDoor && pDoor > pFence && pDoor > pNoEntryLine && pDoor > pOthers)
                    pFreeSpace = normConstHit_ * mapResolution_ - pDoor;
                else if (pGlassDoor > pFence && pGlassDoor > pNoEntryLine && pGlassDoor > pOthers)
                    pFreeSpace = normConstHit_ * mapResolution_ - pGlassDoor;
                else if (pFence > pNoEntryLine && pFence > pFence)
                    pFreeSpace = normConstHit_ * mapResolution_ - pFence;
                else if (pNoEntryLine > pOthers)
                    pFreeSpace = normConstHit_ * mapResolution_ - pNoEntryLine;
                else
                    pFreeSpace = normConstHit_ * mapResolution_ - pOthers;
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR] *= pDoor;
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] *= pGlassDoor;
                spatialLineObjectPrior[(int)LineObjectLabel::CLOSE_GLASS_DOOR] *= pGlassDoor;
                spatialLineObjectPrior[(int)LineObjectLabel::FENCE] *= pFence;
                spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY] *= pNoEntryLine;
                spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE] *= pFreeSpace;
                spatialLineObjectPrior[(int)LineObjectLabel::OTHERS] *= pOthers;

                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR]);
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR]);
                spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::OPEN_GLASS_DOOR]);
                spatialLineObjectPrior[(int)LineObjectLabel::FENCE] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::FENCE]);
                spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::NO_ENTRY]);
                spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE]);
                spatialLineObjectPrior[(int)LineObjectLabel::OTHERS] = clipProb(spatialLineObjectPrior[(int)LineObjectLabel::OTHERS]);

                recog_.normalizeProb(spatialLineObjectPrior);

/*
                printf("%lf: %lf, %lf, %lf, %lf\n", l, pDoor, pFence, pUnobservableWall, pFreeSpace);
                printf("%f, %f, %f\n", doorDist, fenceDist, unobservableWallDist);
                printf("%lf, %lf, %lf, %lf\n", spatialLineObjectPrior[(int)LineObjectLabel::OPEN_DOOR], spatialLineObjectPrior[(int)LineObjectLabel::FENCE],
                    spatialLineObjectPrior[(int)LineObjectLabel::UNOBSERVABLE_WALL], spatialLineObjectPrior[(int)LineObjectLabel::FREE_SPACE]);
                printf("\n");
 */
            }
            xx1 += ddx;
            yy1 += ddy;
        }
        return spatialLineObjectPrior;
    }
}; // class SLAMER : public MCL

} // namespace als_ros

#endif // __SLAMER_H__
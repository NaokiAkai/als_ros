/****************************************************************************
 * als_ros: Advanced Localization Systems for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * To implement this program, a following paper was referred.
 * https://arxiv.org/pdf/1908.01863.pdf
 * 
 * @author Naoki Akai
 ****************************************************************************/

#ifndef __GL_POSE_SAMPLER_H__
#define __GL_POSE_SAMPLER_H__

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <opencv2/opencv.hpp>
#include <als_ros/Pose.h>

namespace als_ros {

class Keypoint {
private:
    int u_, v_;
    double x_, y_;
    char type_;
    // type values -2, -1, 0, or 1.
    // -2: invalid, -1: local minima, 0: suddle, 1: local maxima

public:
    Keypoint(void):
        u_(0), v_(0), x_(0.0), y_(0.0), type_(-2) {}

    Keypoint(int u, int v):
        u_(u), v_(v), x_(0.0), y_(0.0), type_(-2) {}

    Keypoint(double x, double y):
        u_(0), v_(0), x_(x), y_(y), type_(-2) {}

    Keypoint(int u, int v, double x, double y):
        u_(u), v_(v), x_(x), y_(y), type_(-2) {}

    Keypoint(int u, int v, double x, double y, char type):
        u_(u), v_(v), x_(x), y_(y), type_(type) {}

    inline int getU(void) { return u_; }
    inline int getV(void) { return v_; }
    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline char getType(void) { return type_; }

    inline void setU(int u) { u_ = u; }
    inline void setV(int v) { v_ = v; }
    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setType(char type) { type_ = type; }
}; // class Keypoint

class SDFOrientationFeature {
private:
    double dominantOrientation_;
    double averageSDF_;
    std::vector<int> relativeOrientationHist_;

public:
    SDFOrientationFeature(void) {}

    SDFOrientationFeature(double dominantOrientation, double averageSDF, std::vector<int> relativeOrientationHist):
        dominantOrientation_(dominantOrientation),
        averageSDF_(averageSDF),
        relativeOrientationHist_(relativeOrientationHist) {}

    inline double getDominantOrientation(void) { return dominantOrientation_; }
    inline double getAverageSDF(void) { return averageSDF_; }
    std::vector<int> getRelativeOrientationHist(void) { return relativeOrientationHist_; }
    int getRelativeOrientationHist(int idx) { return relativeOrientationHist_[idx]; }
}; // class SDFOrientationFeature

class GLPoseSampler {
private:
    ros::NodeHandle nh_;

    std::string mapName_, scanName_, odomName_, posesName_, localMapName_, sdfKeypointsName_, localSDFKeypointsName_;
    ros::Subscriber mapSub_, scanSub_, odomSub_;
    ros::Publisher posesPub_, localMapPub_, sdfKeypointsPub_, localSDFKeypointsPub_;

    std::string mapFrame_, odomFrame_, baseLinkFrame_, laserFrame_;
    Pose baseLink2Laser_;

    int mapWidth_, mapHeight_;
    double mapResolution_;
    Pose mapOrigin_;
    std::vector<signed char> mapData_;
    bool gotMap_;

    sensor_msgs::LaserScan scan_;
    double keyScanIntervalDist_, keyScanIntervalYaw_;
    std::vector<sensor_msgs::LaserScan> keyScans_;
    int keyScansNum_;

    Pose odomPose_;
    std::vector<Pose> keyPoses_;
    bool gotOdom_;

    std::vector<Keypoint> sdfKeypoints_;
    std::vector<SDFOrientationFeature> sdfOrientationFeatures_;
    visualization_msgs::Marker sdfKeypointsMarker_;

    double gradientSquareTH_;
    double keypointsMinDistFromMap_;
    double sdfFeatureWindowSize_;
    double averageSDFDeltaTH_;
    bool addRandomSamples_, addOppositeSamples_;
    int randomSamplesNum_;
    double positionalRandomNoise_, angularRandomNoise_, matchingRateTH_;

    tf::TransformBroadcaster tfBroadcaster_;
    tf::TransformListener tfListener_;

public:
    GLPoseSampler(void):
        nh_("~"),
        mapName_("/map"),
        scanName_("/scan"),
        odomName_("/odom"),
        posesName_("/gl_sampled_poses"),
        localMapName_("/gl_local_map"),
        sdfKeypointsName_("/gl_sdf_keypoints"),
        localSDFKeypointsName_("/gl_local_sdf_keypoints"),
        mapFrame_("map"),
        odomFrame_("odom"),
        baseLinkFrame_("base_link"),
        laserFrame_("laser"),
        keyScansNum_(5),
        keyScanIntervalDist_(0.5),
        keyScanIntervalYaw_(5.0),
        gradientSquareTH_(10e-4),
        keypointsMinDistFromMap_(1.0),
        sdfFeatureWindowSize_(1.0),
        averageSDFDeltaTH_(1.0),
        addRandomSamples_(true),
        addOppositeSamples_(true),
        randomSamplesNum_(10),
        positionalRandomNoise_(0.5),
        angularRandomNoise_(0.3),
        matchingRateTH_(0.1),
        gotMap_(false),
        gotOdom_(false),
        tfListener_()
    {
        nh_.param("map_name", mapName_, mapName_);
        nh_.param("scan_name", scanName_, scanName_);
        nh_.param("odom_name", odomName_, odomName_);
        nh_.param("poses_name", posesName_, posesName_);
        nh_.param("local_map_name", localMapName_, localMapName_);
        nh_.param("sdf_keypoints_name", sdfKeypointsName_, sdfKeypointsName_);
        nh_.param("local_sdf_keypoints_name", localSDFKeypointsName_, localSDFKeypointsName_);
        nh_.param("map_frame", mapFrame_, mapFrame_);
        nh_.param("odom_frame", odomFrame_, odomFrame_);
        nh_.param("base_link_frame", baseLinkFrame_, baseLinkFrame_);
        nh_.param("laser_frame", laserFrame_, laserFrame_);
        nh_.param("key_scans_num", keyScansNum_, keyScansNum_);
        nh_.param("key_scan_interval_dist", keyScanIntervalDist_, keyScanIntervalDist_);
        nh_.param("key_scan_interval_yaw", keyScanIntervalYaw_, keyScanIntervalYaw_);
        nh_.param("gradient_square_th", gradientSquareTH_, gradientSquareTH_);
        nh_.param("keypoints_min_dist_from_map", keypointsMinDistFromMap_, keypointsMinDistFromMap_);
        nh_.param("sdf_feature_window_size", sdfFeatureWindowSize_, sdfFeatureWindowSize_);
        nh_.param("average_sdf_delta_th", averageSDFDeltaTH_, averageSDFDeltaTH_);
        nh_.param("add_random_samples", addRandomSamples_, addRandomSamples_);
        nh_.param("add_opposite_samples", addOppositeSamples_, addOppositeSamples_);
        nh_.param("random_samples_num", randomSamplesNum_, randomSamplesNum_);
        nh_.param("positional_random_noise", positionalRandomNoise_, positionalRandomNoise_);
        nh_.param("angular_random_noise", angularRandomNoise_, angularRandomNoise_);
        nh_.param("matching_rate_th", matchingRateTH_, matchingRateTH_);

        keyScanIntervalYaw_ *= M_PI / 180.0;

        mapSub_ = nh_.subscribe(mapName_, 1, &GLPoseSampler::mapCB, this);
        odomSub_ = nh_.subscribe(odomName_, 1, &GLPoseSampler::odomCB, this);
        scanSub_ = nh_.subscribe(scanName_, 1, &GLPoseSampler::scanCB, this);

        posesPub_ = nh_.advertise<geometry_msgs::PoseArray>(posesName_, 1);
        localMapPub_ = nh_.advertise<nav_msgs::OccupancyGrid>(localMapName_, 1);
        sdfKeypointsPub_ = nh_.advertise<visualization_msgs::Marker>(sdfKeypointsName_, 1);
        localSDFKeypointsPub_ = nh_.advertise<visualization_msgs::Marker>(localSDFKeypointsName_, 1);

        odomPose_.setPose(0.0, 0.0, 0.0);

        ros::Rate loopRate(10.0);
        int cnt;
        cnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            cnt++;
            if (cnt >= 300) {
                ROS_ERROR("A map message might not be published.");
                exit(1);
            }
            if (gotMap_)
                break;
            loopRate.sleep();
        }

        cnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            cnt++;
            if (cnt >= 300) {
                ROS_ERROR("A odom message might not be published.");
                exit(1);
            }
            if (gotOdom_)
                break;
            loopRate.sleep();
        }

        tf::StampedTransform tfBaseLink2Laser;
        cnt = 0;
        while (ros::ok()) {
            ros::spinOnce();
            try {
                ros::Time now = ros::Time::now();
                tfListener_.waitForTransform(baseLinkFrame_, laserFrame_, now, ros::Duration(2.0));
                tfListener_.lookupTransform(baseLinkFrame_, laserFrame_, now, tfBaseLink2Laser);
                break;
            } catch (tf::TransformException ex) {
                cnt++;
                if (cnt >= 300) {
                    ROS_ERROR("Cannot get the relative pose from the base link to the laser from the tf tree."
                        " Did you set the static transform publisher between %s to %s?",
                        baseLinkFrame_.c_str(), laserFrame_.c_str());
                    exit(1);
                }
                loopRate.sleep();
            }
        }
        tf::Quaternion quatBaseLink2Laser(tfBaseLink2Laser.getRotation().x(),
            tfBaseLink2Laser.getRotation().y(),
            tfBaseLink2Laser.getRotation().z(),
            tfBaseLink2Laser.getRotation().w());
        double baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw;
        tf::Matrix3x3 rotMatBaseLink2Laser(quatBaseLink2Laser);
        rotMatBaseLink2Laser.getRPY(baseLink2LaserRoll, baseLink2LaserPitch, baseLink2LaserYaw);
        baseLink2Laser_.setX(tfBaseLink2Laser.getOrigin().x());
        baseLink2Laser_.setY(tfBaseLink2Laser.getOrigin().y());
        baseLink2Laser_.setYaw(baseLink2LaserYaw);

        ROS_INFO("GL pose sampler ready to perform.");
    }

    void spin(void) {
        ros::spin();
    }

private:
    inline double nrand(double n) {
        return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
    }

    inline void xy2uv(double x, double y, int *u, int *v) {
        double dx = x - mapOrigin_.getX();
        double dy = y - mapOrigin_.getY();
        double yaw = -mapOrigin_.getYaw();
        double xx = dx * cos(yaw) - dy * sin(yaw);
        double yy = dx * sin(yaw) + dy * cos(yaw);
        *u = (int)(xx / mapResolution_);
        *v = (int)(yy / mapResolution_);
    }

    inline void uv2xy(int u, int v, double *x, double *y) {
        double xx = (double)u * mapResolution_;
        double yy = (double)v * mapResolution_;
        double yaw = mapOrigin_.getYaw();
        double dx = xx * cos(yaw) - yy * sin(yaw);
        double dy = xx * sin(yaw) + yy * cos(yaw);
        *x = dx + mapOrigin_.getX();
        *y = dy + mapOrigin_.getY();
    }

    void setMapInfo(nav_msgs::OccupancyGrid map) {
        mapWidth_ = map.info.width;
        mapHeight_ = map.info.height;
        mapResolution_ = map.info.resolution;
        mapOrigin_.setX(map.info.origin.position.x);
        mapOrigin_.setY(map.info.origin.position.y);
        tf::Quaternion q(map.info.origin.orientation.x, 
            map.info.origin.orientation.y, 
            map.info.origin.orientation.z,
            map.info.origin.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mapOrigin_.setYaw(yaw);
        mapData_ = map.data;
    }

    cv::Mat buildDistanceFieldMap(nav_msgs::OccupancyGrid map) {
        cv::Mat binMap(map.info.height, map.info.width, CV_8UC1);
        for (int v = 0; v < map.info.height; v++) {
            for (int u = 0; u < map.info.width; u++) {
                int n = v * map.info.width + u;
                int val = map.data[n];
                if (val == 100)
                    binMap.at<uchar>(v, u) = 0;
                else
                    binMap.at<uchar>(v, u) = 1;
            }
        }

        cv::Mat distMap(map.info.height, map.info.width, CV_32FC1);
        cv::distanceTransform(binMap, distMap, cv::DIST_L2, 5);
        for (int v = 0; v < map.info.height; v++) {
            for (int u = 0; u < map.info.width; u++) {
                float d = distMap.at<float>(v, u) * (float)map.info.resolution;
                distMap.at<float>(v, u) = d;
            }
        }
        return distMap;
    }

    std::vector<Keypoint> detectKeypoints(nav_msgs::OccupancyGrid map, cv::Mat distMap) {
        std::vector<Keypoint> keypoints;
        tf::Quaternion q(map.info.origin.orientation.x, 
            map.info.origin.orientation.y, 
            map.info.origin.orientation.z,
            map.info.origin.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        mapOrigin_.setYaw(yaw);
        for (int u = 1; u < map.info.width - 1; ++u) {
            for (int v = 1; v < map.info.height - 1; ++v) {
                int n = v * map.info.width + u;
                if (map.data[n] != 0 || distMap.at<float>(v, u) < keypointsMinDistFromMap_)
                    continue;

                float dx = -distMap.at<float>(v - 1, u - 1) - distMap.at<float>(v, u - 1) - distMap.at<float>(v + 1, u - 1)
                            + distMap.at<float>(v - 1, u + 1) + distMap.at<float>(v, u + 1) + distMap.at<float>(v + 1, u + 1);
                float dy = -distMap.at<float>(v - 1, u - 1) - distMap.at<float>(v - 1, u) - distMap.at<float>(v - 1, u + 1)
                            + distMap.at<float>(v + 1, u - 1) + distMap.at<float>(v + 1, u) + distMap.at<float>(v + 1, u + 1);

                float dxx = distMap.at<float>(v, u - 1) - 2.0 * distMap.at<float>(v, u) + distMap.at<float>(v, u + 1);
                float dyy = distMap.at<float>(v - 1, u) - 2.0 * distMap.at<float>(v, u) + distMap.at<float>(v + 1, u);
                float dxy = distMap.at<float>(v - 1, u - 1) - distMap.at<float>(v - 1, u)
                            - distMap.at<float>(v, u - 1) + 2.0 * distMap.at<float>(v, u) - distMap.at<float>(v, u + 1)
                            - distMap.at<float>(v + 1, u) + distMap.at<float>(v + 1, u + 1);
                float det = dxx * dyy - dxy * dxy;

                if (dx * dx < gradientSquareTH_ && dy * dy < gradientSquareTH_) {
                    double xx = (double)u * map.info.resolution;
                    double yy = (double)v * map.info.resolution;
                    double dx = xx * cos(yaw) - yy * sin(yaw);
                    double dy = xx * sin(yaw) + yy * cos(yaw);
                    double x = dx + map.info.origin.position.x;
                    double y = dy + map.info.origin.position.y;
                    if (det > 0.0 && dxx < 0.0) // local maxima
                        keypoints.push_back(Keypoint(u, v, x, y, 1));
                    else if (det > 0.0 && dxx > 0.0) // local minima
                        keypoints.push_back(Keypoint(u, v, x, y, -1));
                    else if (det < 0.0) // suddle
                        keypoints.push_back(Keypoint(u, v, x, y, 0));
                }
            }
        }
        return keypoints;
    }

    std::vector<SDFOrientationFeature> calculateFeatures(cv::Mat distMap, std::vector<Keypoint> keypoints) {
        std::vector<SDFOrientationFeature> features((int)keypoints.size());
        for (int i = 0; i < (int)keypoints.size(); ++i) {
            int r = (int)(sdfFeatureWindowSize_ / mapResolution_);
            int uo = keypoints[i].getU();
            int vo = keypoints[i].getV();
            float distSum = 0.0f;
            int cellNum = 0;
            std::vector<int> orientHist(36);
            std::vector<double> orientations;

            for (int u = uo - r; u <= uo + r; ++u) {
                for (int v = vo - r; v <= vo + r; ++v) {
                    if (u < 1 || distMap.cols - 1 < u || v < 1 || distMap.rows - 1 < v)
                        continue;

                    distSum += distMap.at<float>(v, u);
                    cellNum++;

                    float dx = -distMap.at<float>(v - 1, u - 1) - distMap.at<float>(v, u - 1) - distMap.at<float>(v + 1, u - 1)
                                + distMap.at<float>(v - 1, u + 1) + distMap.at<float>(v, u + 1) + distMap.at<float>(v + 1, u + 1);
                    float dy = -distMap.at<float>(v - 1, u - 1) - distMap.at<float>(v - 1, u) - distMap.at<float>(v - 1, u + 1)
                                + distMap.at<float>(v + 1, u - 1) + distMap.at<float>(v + 1, u) + distMap.at<float>(v + 1, u + 1);
                    double t = atan2((double)dy, (double)dx) * 180.0 / M_PI;
                    if (t < 0.0)
                        t += 360.0;
                    int orientIdx = (int)(t / 10.0);
                    if (0 <= orientIdx && orientIdx < 36) {
                        orientHist[orientIdx]++;
                        orientations.push_back(t);
                    }
                }
            }

            float distAve = distSum / (float)cellNum;

            int maxVal = orientHist[0];
            double domOrient = 0.0;
            for (int j = 1; j < (int)orientHist.size(); ++j) {
                if (orientHist[j] > maxVal) {
                    maxVal = orientHist[j];
                    domOrient = (double)j * 10.0;
                }
            }

            std::vector<int> relOrientHist(17);
            for (int j = 0; j < (int)orientations.size(); ++j) {
                double dt = domOrient - orientations[j];
                while (dt > 180.0)
                    dt -= 360.0;
                while (dt < -180.0)
                    dt += 360.0;
                int relOrientIdx = (int)(fabs(dt) / 10.0);
                if (0 <= relOrientIdx && relOrientIdx < 17)
                    relOrientHist[relOrientIdx]++;
            }

            SDFOrientationFeature feature(domOrient * M_PI / 180.0, (double)distAve, relOrientHist);
            features[i] = feature;
        }
        return features;
    }

    visualization_msgs::Marker makeSDFKeypointsMarker(std::vector<Keypoint> keypoints, std::string frame) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.ns = "gl_marker_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        std_msgs::ColorRGBA c;
        c.a = 1.0;
        marker.points.resize((int)keypoints.size());
        marker.colors.resize((int)keypoints.size());
        for (int i = 0; i < (int)keypoints.size(); ++i) {
            geometry_msgs::Point p;
            p.x = keypoints[i].getX();
            p.y = keypoints[i].getY();
            p.z = 0.0;
            char type = keypoints[i].getType();
            if (type == 1)
                c.r = 1.0, c.g = 0.0, c.b = 1.0;
            else if (type == -1)
                c.r = 0.0, c.g = 1.0, c.b = 1.0;
            else
                c.r = 1.0, c.g = 1.0, c.b = 0.0;
            marker.points[i] = p;
            marker.colors[i] = c;
        }
        return marker;
    }

    // it does not correspond to rotation of the map
    void writeMapAndKeypoints(nav_msgs::OccupancyGrid map, std::vector<Keypoint> keypoints) {
        FILE *fp;
        fp = fopen("/tmp/als_ros_map_points.txt", "w");
        for (int u = 0; u < map.info.width; ++u) {
            for (int v = 0; v < map.info.height; ++v) {
                int n = v * map.info.width + u;
                if (map.data[n] == 100) {
                    double x = (double)u * map.info.resolution + map.info.origin.position.x;
                    double y = (double)v * map.info.resolution + map.info.origin.position.y;
                    fprintf(fp, "%lf %lf\n", x, y);
                }
            }
        }
        fclose(fp);

        fp = fopen("/tmp/als_ros_fs_keypoints.txt", "w");
        for (int i = 0; i < (int)keypoints.size(); ++i) {
            double x = (double)keypoints[i].getU() * map.info.resolution + map.info.origin.position.x;
            double y = (double)keypoints[i].getV() * map.info.resolution + map.info.origin.position.y;
//            double x = keypoints[i].getX();
//            double y = keypoints[i].getY();
            fprintf(fp, "%lf %lf %d\n", x, y, keypoints[i].getType());
        }
        fclose(fp);
    }

    void mapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
        setMapInfo(*msg);
        cv::Mat distMap = buildDistanceFieldMap(*msg);
        cv::GaussianBlur(distMap, distMap, cv::Size(5, 5), 5);
        sdfKeypoints_ = detectKeypoints(*msg, distMap);
        sdfOrientationFeatures_ = calculateFeatures(distMap, sdfKeypoints_);
        sdfKeypointsMarker_ = makeSDFKeypointsMarker(sdfKeypoints_, mapFrame_);
        sdfKeypointsPub_.publish(sdfKeypointsMarker_);
//        writeMapAndKeypoints(*msg, sdfKeypoints_);
        gotMap_ = true;
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr &msg) {
        tf::Quaternion q(msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y, 
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        odomPose_.setPose(msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
        gotOdom_ = true;
    }

    nav_msgs::OccupancyGrid buildLocalMap(void) {
        nav_msgs::OccupancyGrid map;
        map.header.frame_id = odomFrame_;

        double rangeMax = keyScans_[0].range_max;
        map.info.width = (int)(rangeMax * 3.0 / mapResolution_);
        map.info.height = (int)(rangeMax * 3.0 / mapResolution_);
        map.info.resolution = mapResolution_;
        map.info.origin.position.x = keyPoses_[0].getX() - rangeMax * 1.5;
        map.info.origin.position.y = keyPoses_[0].getY() - rangeMax * 1.5;
        map.info.origin.orientation.w = 1.0;
        map.data.resize(map.info.width * map.info.height, -1);

        for (int i = 0; i < (int)keyScans_.size(); ++i) {
            double yaw = baseLink2Laser_.getYaw();
            double c = cos(yaw);
            double s = sin(yaw);
            double sensorX = baseLink2Laser_.getX() * c - baseLink2Laser_.getY() * s + keyPoses_[i].getX();
            double sensorY = baseLink2Laser_.getX() * s + baseLink2Laser_.getY() * c + keyPoses_[i].getY();
            double sensorYaw = yaw + keyPoses_[i].getYaw();
            sensor_msgs::LaserScan scan = keyScans_[i];
            for (int j = 0; j < (int)scan.ranges.size(); ++j) {
                double range = scan.ranges[j];
                if (range < scan.range_min || scan.range_max < range)
                    continue;
                if (range < keypointsMinDistFromMap_)
                    continue;

                double t = (double)j * scan.angle_increment + scan.angle_min + sensorYaw;
                double x = sensorX;
                double y = sensorY;
                double dx = mapResolution_ * cos(t);
                double dy = mapResolution_ * sin(t);
                for (double r = 0.0; r < range - mapResolution_; r += mapResolution_) {
                    int u = (int)((x - map.info.origin.position.x) / map.info.resolution);
                    int v = (int)((y - map.info.origin.position.y) / map.info.resolution);
                    if (0 < u && u < map.info.width && 0 < v && v < map.info.height) {
                        int n = v * map.info.width + u;
                        map.data[n] = 0;
                    }
                    x += dx;
                    y += dy;
                }
                x = range * cos(t) + sensorX;
                y = range * sin(t) + sensorY;
                int u = (int)((x - map.info.origin.position.x) / map.info.resolution);
                int v = (int)((y - map.info.origin.position.y) / map.info.resolution);
                if (0 < u && u < map.info.width && 0 < v && v < map.info.height) {
                    int n = v * map.info.width + u;
                    map.data[n] = 100;
                }
            }
        }
        return map;
    }

    std::vector<int> findCorrespondingFeatures(std::vector<Keypoint> localSDFKeypoints, std::vector<SDFOrientationFeature> localFeatures) {
        std::vector<int> correspondingIndices((int)localSDFKeypoints.size());
        for (int i = 0; i < (int)localSDFKeypoints.size(); ++i) {
            char localKeypointType = localSDFKeypoints[i].getType();
            double localAverageSDF = localFeatures[i].getAverageSDF();
            std::vector<int> localRelOrientHist = localFeatures[i].getRelativeOrientationHist();
            bool isFirst = true, isSecond = true;
            int idx1, idx2, min1 = -1, min2 = -1;
            for (int j = 0; j < (int)sdfKeypoints_.size(); ++j) {
//                if (localKeypointType != sdfKeypoints_[j].getType() || localKeypointType == 0)
                if (localKeypointType != sdfKeypoints_[j].getType())
                    continue;
                double dAverageSDF = localAverageSDF - sdfOrientationFeatures_[j].getAverageSDF();
                if (fabs(dAverageSDF) > averageSDFDeltaTH_)
                    continue;

                int sum = 0;
                for (int k = 0; k < 17; ++k)
                    sum += abs(localRelOrientHist[k] - sdfOrientationFeatures_[j].getRelativeOrientationHist(k));

                if (isFirst) {
                    min1 = sum;
                    idx1 = j;
                    isFirst = false;
                } else if (isSecond) {
                    if (min1 < sum) {
                        min2 = sum;
                        idx2 = j;
                    } else {
                        min2 = min1;
                        idx2 = idx1;
                        min1 = sum;
                        idx1 = j;
                    }
                    isSecond = false;
                } else if (min1 > sum) {
                    min2 = min1;
                    idx2 = idx1;
                    min1 = sum;
                    idx1 = j;
                }
            }

            if (min1 >= 0 && min2 >= 0 && (float)min1 * 1.5f < (float)min2)
                correspondingIndices[i] = idx1;
            else if (min1 >= 0 && min2 < 0)
                correspondingIndices[i] = idx1;
            else
                correspondingIndices[i] = -1;
        }
        return correspondingIndices;
    }

    double computeMatchingRate(Pose pose) {
        double yaw = baseLink2Laser_.getYaw();
        double c = cos(yaw);
        double s = sin(yaw);
        double sensorX = baseLink2Laser_.getX() * c - baseLink2Laser_.getY() * s + pose.getX();
        double sensorY = baseLink2Laser_.getX() * s + baseLink2Laser_.getY() * c + pose.getY();
        double sensorYaw = yaw + pose.getYaw();
        sensor_msgs::LaserScan scan = keyScans_[(int)keyScans_.size() - 1];

        int validScanNum = 0, matchingNum = 0;
        for (int i = 0; i < (int)scan.ranges.size(); ++i) {
            double r = scan.ranges[i];
            if (r < scan.range_min || scan.range_max < r)
                continue;
            if (r < keypointsMinDistFromMap_)
                continue;

            validScanNum++;
            double t = (double)i * scan.angle_increment + scan.angle_min + sensorYaw;
            double x = r * cos(t) + sensorX;
            double y = r * sin(t) + sensorY;
            int u, v;
            xy2uv(x, y, &u, &v);
            if (1 <= u && u < mapWidth_ - 1 && 1 <= v && v < mapHeight_ - 1) {
                int n0 = v * mapWidth_ + u;
                int n1 = n0 - mapWidth_;
                int n2 = n0 - 1;
                int n3 = n0 + 1;
                int n4 = n0 + mapWidth_;
                if (mapData_[n0] == 100 || mapData_[n1] == 100 || mapData_[n2] == 100 || mapData_[n3] == 100 || mapData_[n4] == 100)
                    matchingNum++;
            }
        }
        return (double)matchingNum / (double)validScanNum;
    }

    geometry_msgs::PoseArray generatePoses(Pose currentOdomPose, std::vector<Keypoint> localSDFKeypoints,
        std::vector<SDFOrientationFeature> localSDFOrientationFeatures, std::vector<int> correspondingIndices)
    {
        geometry_msgs::PoseArray poses;
        poses.header.frame_id = mapFrame_;
        for (int i = 0; i < (int)correspondingIndices.size(); ++i) {
            int idx = correspondingIndices[i];
            if (idx < 0)
                continue;

            double dx = localSDFKeypoints[i].getX() - currentOdomPose.getX();
            double dy = localSDFKeypoints[i].getY() - currentOdomPose.getY();
            double localDomOrient = localSDFOrientationFeatures[i].getDominantOrientation();
            double dOrient = currentOdomPose.getYaw() - localDomOrient;
            Keypoint targetKeypoint = sdfKeypoints_[idx];
            double targetDomOrient = sdfOrientationFeatures_[idx].getDominantOrientation();

            double dDomOrient = localDomOrient - targetDomOrient;
            double c = cos(dDomOrient);
            double s = sin(dDomOrient);
            double sensorX = dx * c - dy * s + targetKeypoint.getX();
            double sensorY = dx * s + dy * c + targetKeypoint.getY();
            double sensorYaw = targetDomOrient + dOrient;

            int u, v;
            xy2uv(sensorX, sensorY, &u, &v);
            if (u < 0 || mapWidth_ <= u || v < 0 || mapHeight_ <= v)
                continue;
            int n = v * mapWidth_ + u;
            if (mapData_[n] != 0)
                continue;

            double byaw = baseLink2Laser_.getYaw();
            double bc = cos(byaw);
            double bs = sin(byaw);
            double baseX = sensorX - baseLink2Laser_.getX() * bc + baseLink2Laser_.getY() * bs;
            double baseY = sensorY - baseLink2Laser_.getX() * bs - baseLink2Laser_.getY() * bc;
            double baseYaw = sensorYaw - byaw;

            if (!addRandomSamples_) {
                if (matchingRateTH_ > 0.0) {
                    if (computeMatchingRate(Pose(baseX, baseY, baseYaw)) < matchingRateTH_)
                        continue;
                }
                geometry_msgs::Pose pose;
                pose.position.x = baseX;
                pose.position.y = baseY;
                pose.orientation = tf::createQuaternionMsgFromYaw(baseYaw);
                poses.poses.push_back(pose);
            } else {
                for (int j = 0; j < randomSamplesNum_; ++j) {
                    double x = baseX + nrand(positionalRandomNoise_);
                    double y = baseY + nrand(positionalRandomNoise_);
                    double yaw;
                    if (addOppositeSamples_ && j % 2 == 1)
                        yaw = baseYaw + M_PI + nrand(angularRandomNoise_);
                    else
                        yaw = baseYaw + nrand(angularRandomNoise_);
                    if (matchingRateTH_ > 0.0) {
                        if (computeMatchingRate(Pose(x, y, yaw)) < matchingRateTH_)
                            continue;
                    }
                    geometry_msgs::Pose pose;
                    pose.position.x = x;
                    pose.position.y = y;
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    poses.poses.push_back(pose);
                }
            }
        }
        return poses;
    }

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        static bool isFirst = true;
        static Pose prevOdomPose;

        int validScanNum = 0;
        for (int i = 0; i < (int)msg->ranges.size(); ++i) {
            float r = msg->ranges[i];
            if (msg->range_min <= r && r <= msg->range_max)
                validScanNum++;
        }
        double validScanRate = (double)validScanNum / (double)msg->ranges.size();
        if (validScanRate < 0.1) {
            ROS_WARN("gl pose sampler might subscribe invalid scan.");
            return;
        }

        if (isFirst && gotOdom_) {
            keyScans_.push_back(*msg);
            keyPoses_.push_back(odomPose_);
            prevOdomPose.setPose(odomPose_);
            isFirst = false;
            return;
        }

        bool isKeyScanUpdated = false;
        double dx = odomPose_.getX() - prevOdomPose.getX();
        double dy = odomPose_.getY() - prevOdomPose.getY();
        double dl = sqrt(dx * dx + dy * dy);
        double dyaw = odomPose_.getYaw() - prevOdomPose.getYaw();
        while (dyaw < -M_PI)
            dyaw += 2.0 * M_PI;
        while (dyaw > M_PI)
            dyaw -= 2.0 * M_PI;
        if (dl > keyScanIntervalDist_ || fabs(dyaw) > keyScanIntervalYaw_) {
            keyScans_.insert(keyScans_.begin(), *msg);
            keyPoses_.insert(keyPoses_.begin(), odomPose_);
            if ((int)keyScans_.size() >= keyScansNum_) {
                keyScans_.resize(keyScansNum_);
                keyPoses_.resize(keyScansNum_);
            }
            prevOdomPose.setPose(odomPose_);
            isKeyScanUpdated = true;
        }

        if (isKeyScanUpdated && (int)keyScans_.size() == keyScansNum_) {
            nav_msgs::OccupancyGrid localMap = buildLocalMap();
            cv::Mat localDistMap = buildDistanceFieldMap(localMap);
            cv::GaussianBlur(localDistMap, localDistMap, cv::Size(5, 5), 5);
            std::vector<Keypoint> localSDFKeypoints = detectKeypoints(localMap, localDistMap);
            std::vector<SDFOrientationFeature> localSDFOrientationFeatures = calculateFeatures(localDistMap, localSDFKeypoints);
//            writeMapAndKeypoints(localMap, localSDFKeypoints);
            std::vector<int> correspondingIndices = findCorrespondingFeatures(localSDFKeypoints, localSDFOrientationFeatures);
            geometry_msgs::PoseArray poses = generatePoses(prevOdomPose, localSDFKeypoints, localSDFOrientationFeatures, correspondingIndices);
            visualization_msgs::Marker localSDFKeypointsMarker = makeSDFKeypointsMarker(localSDFKeypoints, odomFrame_);

            poses.header.stamp = localMap.header.stamp = sdfKeypointsMarker_.header.stamp = localSDFKeypointsMarker.header.stamp = msg->header.stamp;
            posesPub_.publish(poses);
            localMapPub_.publish(localMap);
            sdfKeypointsPub_.publish(sdfKeypointsMarker_);
            localSDFKeypointsPub_.publish(localSDFKeypointsMarker);
        }
    }
}; // class GLPoseSampler

} // namespace als_ros

#endif // __GL_POSE_SAMPLER_H__

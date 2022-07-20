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

#ifndef __ISM_H__
#define __ISM_H__

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <als_ros/Pose.h>

namespace als_ros {

class ISM {
private:
    std::vector<std::string> objectNames_;
    std::vector<std::vector<int>> objectColors_;

    int mapWidth_, mapHeight_;
    double mapResolution_;
    Pose mapOrigin_;

    std::vector<cv::Mat> distMaps_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ismPoints_;

public:
    ISM(void):
        ismPoints_(new pcl::PointCloud<pcl::PointXYZRGB>()) {}

    inline std::vector<std::string> getObjectNames(void) { return objectNames_; }
    inline double getMapResolution(void) { return mapResolution_; }

    void readISM(std::string ismYamlFile) {
        std::string ismYamlPath = (std::string)realpath(ismYamlFile.c_str(), NULL);
        std::string dirPath = ismYamlPath.substr(0, ismYamlPath.find_last_of("/") + 1);
        // std::cout << "ISM YAML payh is " << ismYamlPath << std::endl;
        // std::cout << "Dir path is " << dirPath << std::endl;
        YAML::Node ismNode = YAML::LoadFile(ismYamlFile);

        std::string ogmInfoYamlFile = ismNode["ogm_info_yaml"].as<std::string>();
        std::string ogmFilePath = dirPath + ogmInfoYamlFile;
        YAML::Node ogmNode = YAML::LoadFile(ogmFilePath);
        mapResolution_ = ogmNode["resolution"].as<double>();
        std::vector<double> mapOrigin = ogmNode["origin"].as<std::vector<double>>();
        mapOrigin_.setPose(mapOrigin[0], mapOrigin[1], mapOrigin[2]);

        std::string ismImageFile = ismNode["image"].as<std::string>();
        std::string ismFilePath = dirPath + ismImageFile;
        // std::cout << "ISM image file path is " << ismFilePath << std::endl;
        cv::Mat ismImage = cv::imread(ismFilePath.c_str(), 1);
        if (ismImage.empty()) {
            ROS_ERROR("Could not read the ISM image -> %s", ismFilePath.c_str());
            exit(1);
        }
        mapWidth_ = ismImage.cols;
        mapHeight_ = ismImage.rows;
        // std::cout << "mapWidth_ " << mapWidth_ << std::endl;
        // std::cout << "mapHeight_ " << mapHeight_ << std::endl;

        objectNames_ = ismNode["objects"].as<std::vector<std::string>>();
        objectColors_.resize((int)objectNames_.size());
        for (int i = 0; i < (int)objectNames_.size(); ++i) {
            std::string tagName = objectNames_[i] + "_color";
            objectColors_[i] = ismNode[tagName.c_str()].as<std::vector<int>>();
            // std::cout << i << " " << objectColors_[i][0] << " " << objectColors_[i][1] << " " << objectColors_[i][2] << std::endl;
        }

        std::vector<cv::Mat> objectMaps((int)objectNames_.size());
        for (int i = 0; i < (int)objectMaps.size(); ++i)
            objectMaps[i] = cv::Mat::ones(mapHeight_, mapWidth_, CV_8UC1);

        for (int u = 0; u < ismImage.cols; ++u) {
            for (int v = 0; v < ismImage.rows; ++v) {
                cv::Vec3b val = ismImage.at<cv::Vec3b>(v, u);
                if (val[0] == 205 && val[1] == 205 && val[2] == 205) // unknown space
                    continue;
                if (val[0] == 254 && val[1] == 254 && val[2] == 254) // free space
                    continue;
                // if (val[0] == 0 && val[1] == 0 && val[2] == 0) // occupied space
                //     continue;
                // NOTE: occupied space is treated as others in slamer
                for (int i = 0; i < (int)objectNames_.size(); ++i) {
                    if (val[0] == objectColors_[i][2] && val[1] == objectColors_[i][1] && val[2] == objectColors_[i][0]) {
                        pcl::PointXYZRGB p;
                        uv2xy<float>(u, v, &p.x, &p.y);
                        p.z = 0.0;
                        p.r = objectColors_[i][0];
                        p.g = objectColors_[i][1];
                        p.b = objectColors_[i][2];
                        ismPoints_->points.push_back(p);
                        objectMaps[i].at<uchar>(v, u) = 0;
                        // printf("%s, at (u, v) = (%d, %d) with (r, g, b) = (%d, %d, %d)\n",
                        //     objectNames_[i].c_str(), u, v, val[2], val[1], val[0]);
                        break;
                    }
                }
            }
        }
        ismPoints_->width = ismPoints_->points.size();
        ismPoints_->height = 1;

        distMaps_.resize((int)objectMaps.size());
        for (int i = 0; i < (int)objectMaps.size(); ++i) {
            distMaps_[i] = cv::Mat(mapHeight_, mapWidth_, CV_32FC1);
            cv::distanceTransform(objectMaps[i], distMaps_[i], cv::DIST_L2, 5);
            for (int v = 0; v < mapHeight_; v++) {
                for (int u = 0; u < mapWidth_; u++) {
                    float d = distMaps_[i].at<float>(v, u) * (float)mapResolution_;
                    distMaps_[i].at<float>(v, u) = d;
                }
            }
        }
    }

    inline sensor_msgs::PointCloud2 getISMPointsAsPC2(void) {
        sensor_msgs::PointCloud2 ismPointsMsg;
        pcl::toROSMsg(*ismPoints_.get(), ismPointsMsg);
        return ismPointsMsg;
    }

    inline float getDistance(double x, double y, std::string object) {
        int u, v;
        xy2uv<double>(x, y, &u, &v);
        if (u < 0 || mapWidth_ <= u || v < 0 || mapHeight_ <= v)
            return -1.0f;

        for (int i = 0; i < (int)objectNames_.size(); ++i) {
            if (objectNames_[i] == object)
                return distMaps_[i].at<float>(v, u);
        }
        return -1.0f;
    }

private:
    template <typename T>
    inline void xy2uv(T x, T y, int *u, int *v) {
        *u = (int)((x - mapOrigin_.getX()) / mapResolution_);
        *v = mapHeight_ - 1 - (int)((y - mapOrigin_.getY()) / mapResolution_);
    }

    template <typename T>
    inline void uv2xy(int u, int v, T *x, T *y) {
        *x = (T)u * mapResolution_ + mapOrigin_.getX();
        *y = (T)(mapHeight_ - 1 - v) * mapResolution_ + mapOrigin_.getY();
    }
}; // class ISM

} // namespace als_ros

#endif // __ISM_H__
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

#ifndef __POSE_H__
#define __POSE_H__

#include <cmath>

namespace als_ros {

class Pose {
private:
    double x_, y_, yaw_;

    void modifyYaw(void) {
        while (yaw_ < -M_PI)
            yaw_ += 2.0 * M_PI;
        while (yaw_ > M_PI)
            yaw_ -= 2.0 * M_PI;
    }

public:
    Pose():
        x_(0.0), y_(0.0), yaw_(0.0) {};

    Pose(double x, double y, double yaw):
        x_(x), y_(y), yaw_(yaw) {};

    ~Pose() {};

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setYaw(double yaw) { yaw_ = yaw, modifyYaw(); }
    inline void setPose(double x, double y, double yaw) { x_ = x, y_ = y, yaw_ = yaw, modifyYaw(); }
    inline void setPose(Pose p) { x_ = p.x_, y_ = p.y_, yaw_ = p.yaw_, modifyYaw(); }

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline double getYaw(void) { return yaw_; }
    inline Pose getPose(void) { return Pose(x_, y_, yaw_); }

}; // class Pose

} // namespace als_ros

#endif // __POSE_H__

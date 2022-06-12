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

#ifndef __PARTICLE_H__
#define __PARTICLE_H__

#include <als_ros/Pose.h>

namespace als_ros {

class Particle {
private:
    Pose pose_;
    double w_;

public:
    Particle(): pose_(0.0, 0.0, 0.0), w_(0.0) {};

    Particle(double x, double y, double yaw, double w): pose_(x, y, yaw), w_(w) {};

    Particle(Pose p, double w): pose_(p), w_(w) {};

    ~Particle() {};

    inline double getX(void) { return pose_.getX(); }
    inline double getY(void) { return pose_.getY(); }
    inline double getYaw(void) { return pose_.getYaw(); }
    inline Pose getPose(void) { return pose_; }
    inline double getW(void) { return w_; }

    inline void setX(double x) { pose_.setX(x); }
    inline void setY(double y) { pose_.setY(y); }
    inline void setYaw(double yaw) { pose_.setYaw(yaw); }
    inline void setPose(double x, double y, double yaw) { pose_.setPose(x, y, yaw); }
    inline void setPose(Pose p) { pose_.setPose(p); }
    inline void setW(double w) { w_ = w; }
}; // class Particle

} // namespace als_ros

#endif // __PARTICLE_H__

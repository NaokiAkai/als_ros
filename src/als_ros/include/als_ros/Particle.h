/****************************************************************************
 * als_ros: Advanced Localization Systems for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
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

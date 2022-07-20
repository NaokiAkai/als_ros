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

#ifndef __POINT_H__
#define __POINT_H__

namespace als_ros {

class Point {
private:
    double x_, y_;

public:
    Point():
        x_(0.0), y_(0.0) {};

    Point(double x, double y):
        x_(x), y_(y) {};

    ~Point() {};

    inline void setX(double x) { x_ = x; }
    inline void setY(double y) { y_ = y; }
    inline void setPoint(double x, double y) { x_ = x, y_ = y; }
    inline void setPoint(Point p) { x_ = p.x_, y_ = p.y_; }

    inline double getX(void) { return x_; }
    inline double getY(void) { return y_; }
    inline Point getPoint(void) { return Point(x_, y_); }

}; // class Point

} // namespace als_ros

#endif // __POINT_H__

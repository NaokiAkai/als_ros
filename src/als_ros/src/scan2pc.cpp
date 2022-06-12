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
#include <sensor_msgs/PointCloud.h>

class Scan2PC {
private:
    ros::NodeHandle nh_;
    std::string scanName_, pcName_;
    ros::Subscriber scanSub_;
    ros::Publisher pcPub_;

public:
    Scan2PC(void):
        nh_("~"),
        scanName_("/scan"),
        pcName_("/scan_point_cloud")
    {
        nh_.param("scan_name", scanName_, scanName_);
        nh_.param("pc_name", pcName_, pcName_);

        scanSub_ = nh_.subscribe(scanName_, 1, &Scan2PC::scanCB, this);
        pcPub_ = nh_.advertise<sensor_msgs::PointCloud>(pcName_, 1);
    }

    void spin(void) {
        ros::spin();
    }

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        sensor_msgs::PointCloud pc;
        pc.header = msg->header;
        for (int i = 0; i < (int)msg->ranges.size(); ++i) {
            double r = msg->ranges[i];
            if (r <= msg->range_min || msg->range_max <= r)
                continue;
            double t = msg->angle_min + (double)i * msg->angle_increment;
            double x = r * cos(t);
            double y = r * sin(t);
            geometry_msgs::Point32 p;
            p.x = x;
            p.y = y;
            p.z = 0.0;
            pc.points.push_back(p);
        }
        pcPub_.publish(pc);
    }
}; // class Scan2PC

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan2pc");
    Scan2PC node;
    node.spin();
    return 0;
}

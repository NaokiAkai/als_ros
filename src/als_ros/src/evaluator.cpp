/****************************************************************************
 * als_ros: An Advanced Localization System for ROS use with 2D LiDAR
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * @author Naoki Akai
 ****************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

typedef struct {
    double x, y;
} Point;

class Evaluator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber gtPoseSub_, reliabilitySub_, glPosesSub_, scanSub_;
    std::string mapFrame_, laserFrame_;
    tf::TransformListener tfListener_;
    std::vector<geometry_msgs::PoseStamped> gtPoses_;
    std::vector<geometry_msgs::PoseArray> glPoses_;
    std::vector<sensor_msgs::LaserScan> scans_;
    bool canUpdateGTPoses_, canUpdateGLPoses_, canUpdateScan_;
    bool saveGTPose_, saveGLPoses_, saveScan_;

public:
    Evaluator(void):
        nh_("~"),
        mapFrame_("map"),
        laserFrame_("laser"),
        canUpdateGTPoses_(true),
        canUpdateGLPoses_(true),
        canUpdateScan_(true),
        tfListener_()
    {
        gtPoseSub_ = nh_.subscribe("/scanned_ground_truth_pose", 1, &Evaluator::gtPoseCB, this);
        reliabilitySub_ = nh_.subscribe("/reliability", 1, &Evaluator::reliabilityCB, this);
        glPosesSub_ = nh_.subscribe("/gl_sampled_poses", 1, &Evaluator::glPosesCB, this);
        scanSub_ = nh_.subscribe("/scan", 1, &Evaluator::scanCB, this);
        saveGTPose_ = false;
        saveGLPoses_ = false;
        saveScan_ = false;
    }

    void spin(void) {
        ros::spin();
    }

    void gtPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg) {
        if (!canUpdateGTPoses_)
            return;
        gtPoses_.insert(gtPoses_.begin(), *msg);
        if (gtPoses_.size() >= 100)
            gtPoses_.resize(100);
    }

    int getSynchronizedGTPoses(double time) {
        for (int i = 0; i < (int)gtPoses_.size(); ++i) {
            double t = gtPoses_[i].header.stamp.toSec();
            if (t < time)
                return i;
        }
        return -1;
    }

    void glPosesCB(const geometry_msgs::PoseArray::ConstPtr &msg) {
        if (!canUpdateGLPoses_)
            return;
        glPoses_.insert(glPoses_.begin(), *msg);
        if (glPoses_.size() >= 100)
            glPoses_.resize(100);
    }

    int getSynchronizedGLPoses(double time) {
        for (int i = 0; i < (int)glPoses_.size(); ++i) {
            double t = glPoses_[i].header.stamp.toSec();
            if (t < time)
                return i;
        }
        return -1;
    }

    void scanCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if (!canUpdateScan_)
            return;
        scans_.insert(scans_.begin(), *msg);
        if (scans_.size() >= 100)
            scans_.resize(100);
    }

    int getSynchronizedScan(double time) {
        for (int i = 0; i < (int)scans_.size(); ++i) {
            double t = scans_[i].header.stamp.toSec();
            if (t < time)
                return i;
        }
        return -1;
    }

    std::vector<Point> makeArrowPoints(double x, double y, double yaw, double len) {
        Point p0, p1, p2, p3;
        p0.x = x;
        p0.y = y;
        p1.x = p0.x + len * cos(yaw);
        p1.y = p0.y + len * sin(yaw);
        p2.x = p1.x + len / 3.0 * cos(yaw + 135.0 * M_PI / 180.0);
        p2.y = p1.y + len / 3.0 * sin(yaw + 135.0 * M_PI / 180.0);
        p3.x = p1.x + len / 3.0 * cos(yaw - 135.0 * M_PI / 180.0);
        p3.y = p1.y + len / 3.0 * sin(yaw - 135.0 * M_PI / 180.0);

        std::vector<Point> arrowPoints;
        arrowPoints.push_back(p0);
        arrowPoints.push_back(p1);
        arrowPoints.push_back(p2);
        arrowPoints.push_back(p3);
        arrowPoints.push_back(p1);
        return arrowPoints;
    }

    void reliabilityCB(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
        static bool isFirst = true;
        static double firstTime;
        static FILE *fp = fopen("/tmp/als_ros_reliability.txt", "w");
        double time = msg->header.stamp.toSec();
        if (isFirst) {
            firstTime = time;
            isFirst = false;
        }

        geometry_msgs::PoseStamped gtPose;
        if (saveGTPose_) {
            canUpdateGTPoses_ = false;
            int gtPoseIdx = getSynchronizedGTPoses(time);
            if (gtPoseIdx < 0) {
                canUpdateGTPoses_ = true;
                return;
            }
            gtPose = gtPoses_[gtPoseIdx];
            canUpdateGTPoses_ = true;
        }

        geometry_msgs::PoseArray glPoses;
        if (saveGLPoses_) {
            canUpdateGLPoses_ = false;
            int glPoseIdx = getSynchronizedGLPoses(time);
            if (glPoseIdx < 0) {
                canUpdateGLPoses_ = true;
                return;
            }
            glPoses = glPoses_[glPoseIdx];
            canUpdateGLPoses_ = true;
        }

        sensor_msgs::LaserScan scan;
        if (saveScan_) {
            canUpdateScan_ = false;
            int scanIdx = getSynchronizedScan(time);
            if (scanIdx < 0) {
                canUpdateScan_ = true;
                return;
            }
            scan = scans_[scanIdx];
            canUpdateScan_ = true;
        }

        tf::StampedTransform tfMap2Laser;
        try {
            ros::Time now = msg->header.stamp;
            tfListener_.waitForTransform(mapFrame_, laserFrame_, now, ros::Duration(0.2));
            tfListener_.lookupTransform(mapFrame_, laserFrame_, now, tfMap2Laser);
        } catch (tf::TransformException ex) {
            return;
        }

        double gtX, gtY, gtYaw;
        if (saveGTPose_) {
            tf::Quaternion gtQuat(gtPose.pose.orientation.x,
                gtPose.pose.orientation.y,
                gtPose.pose.orientation.z,
                gtPose.pose.orientation.w);
            double gtRoll, gtPitch;
            tf::Matrix3x3 gtRotMat(gtQuat);
            gtRotMat.getRPY(gtRoll, gtPitch, gtYaw);
            gtX = gtPose.pose.position.x;
            gtY = gtPose.pose.position.y;
        }

        tf::Quaternion quat(tfMap2Laser.getRotation().x(),
            tfMap2Laser.getRotation().y(),
            tfMap2Laser.getRotation().z(),
            tfMap2Laser.getRotation().w());
        double roll, pitch, yaw;
        tf::Matrix3x3 rotMat(quat);
        rotMat.getRPY(roll, pitch, yaw);
        double x = tfMap2Laser.getOrigin().x();
        double y = tfMap2Laser.getOrigin().y();

        if (saveGTPose_) {
            double dx = gtX - x;
            double dy = gtY - y;
            double dl = sqrt(dx * dx + dy * dy);
            double dyaw = gtYaw - yaw;
            while (dyaw < -M_PI)
                dyaw += 2.0 * M_PI;
            while (dyaw > M_PI)
                dyaw -= 2.0 * M_PI;
            printf("%.2lf [sec], %.3lf [m], %.3lf [m], %.3lf [m], %.3lf [deg], %lf %lf %lf\n",
                time - firstTime, dx, dy, dl, dyaw * 180.0 / M_PI, msg->vector.x, msg->vector.y, msg->vector.z);

            fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                time - firstTime, gtX, gtY, gtYaw, x, y, yaw, 
                dx, dy, dl, dyaw, msg->vector.x, msg->vector.y, msg->vector.z);
        } else {
            printf("%.2lf [sec], %.3lf [m], %.3lf [m], %.3lf [deg], %lf %lf %lf\n",
                time - firstTime, x, y, yaw * 180.0 / M_PI,
                msg->vector.x, msg->vector.y, msg->vector.z);

            fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf\n",
                time - firstTime, x, y, yaw, 
                msg->vector.x, msg->vector.y, msg->vector.z);
        }

        if (saveGLPoses_) {
            FILE *fp2;
            fp2 = fopen("/tmp/als_ros_scaned_ground_truth.txt", "w");
            std::vector<Point> gtArrowPoints = makeArrowPoints(gtX, gtY, gtYaw, 1.0);
            for (int i = 0; i < (int)gtArrowPoints.size(); ++i)
                fprintf(fp2, "%lf %lf\n", gtArrowPoints[i].x, gtArrowPoints[i].y);
            fprintf(fp2, "\n");
            fclose(fp2);

            fp2 = fopen("/tmp/als_ros_mcl_estimate.txt", "w");
            std::vector<Point> mclArrowPoints = makeArrowPoints(x, y, yaw, 1.0);
            for (int i = 0; i < (int)mclArrowPoints.size(); ++i)
                fprintf(fp2, "%lf %lf\n", mclArrowPoints[i].x, mclArrowPoints[i].y);
            fprintf(fp2, "\n");
            fclose(fp2);

            double tmpYaw = 0.0, estX = 0.0, estY = 0.0, estYaw = 0.0;
            double wo = 1.0 / (double)glPoses.poses.size();
            fp2 = fopen("/tmp/als_ros_gl_sampled_poses.txt", "w");
            for (int i = 0; i < (int)glPoses.poses.size(); ++i) {
                tf::Quaternion quat(glPoses.poses[i].orientation.x,
                    glPoses.poses[i].orientation.y,
                    glPoses.poses[i].orientation.z,
                    glPoses.poses[i].orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3 rotMat(quat);
                rotMat.getRPY(roll, pitch, yaw);
                double x = glPoses.poses[i].position.x;
                double y = glPoses.poses[i].position.y;
                std::vector<Point> arrowPoints = makeArrowPoints(x, y, yaw, 1.0);
                for (int j = 0; j < (int)arrowPoints.size(); ++j)
                    fprintf(fp2, "%lf %lf\n", arrowPoints[j].x, arrowPoints[j].y);
                fprintf(fp2, "\n");

                estX += x * wo;
                estY += y * wo;
                double dyaw = tmpYaw - yaw;
                while (dyaw < -M_PI)
                    dyaw += 2.0 * M_PI;
                while (dyaw > M_PI)
                    dyaw -= 2.0 * M_PI;
                estYaw += dyaw * wo;
            }
            fclose(fp2);

            estYaw = tmpYaw - estYaw;
            static FILE *fpGT = fopen("/tmp/als_ros_scaned_ground_truth_poses.txt", "w");
            static FILE *fpEst = fopen("/tmp/als_ros_mcl_poses.txt", "w");
            static FILE *fpGLAve = fopen("/tmp/als_ros_gl_sampled_poses_ave.txt", "w");
            fprintf(fpGT, "%lf %lf %lf\n", gtX, gtY, gtYaw);
            fprintf(fpEst, "%lf %lf %lf\n", x, y, yaw);
            fprintf(fpGLAve, "%lf %lf %lf\n", estX, estY, estYaw);
        }

        if (saveScan_) {
            FILE *fp2 = fopen("/tmp/als_ros_scan_points.txt", "w");
            for (int i = 0; i < (int)scan.ranges.size(); ++i) {
                double r = scan.ranges[i];
                if (r < scan.range_min || scan.range_max < r)
                    continue;
                double t = (double)i * scan.angle_increment + scan.angle_min + gtYaw;
                double x = r * cos(t) + gtX;
                double y = r * sin(t) + gtY;
                fprintf(fp2, "%lf %lf\n", x, y);
            }
            fclose(fp2);
        }
    }
}; // class Evaluator

int main(int argc, char **argv) {
    ros::init(argc, argv, "evaluator");
    Evaluator node;
    node.spin();
    return 0;
}
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

#ifndef __MRF_FAILURE_DETECTOR_H__
#define __MRF_FAILURE_DETECTOR_H__

#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <visualization_msgs/Marker.h>

namespace als_ros {

enum MeasurementClass {
    ALIGNED = 0,
    MISALIGNED = 1,
    UNKNOWN = 2
};

class MRFFD {
private:
    // ros subscribers and publishers
    ros::NodeHandle nh_;
    std::string residualErrorsName_;
    ros::Subscriber residualErrorsSub_;
    std::string failureProbName_, alignedScanName_, misalignedScanName_, unknownScanName_;
    ros::Publisher failureProbPub_, alignedScanPub_, misalignedScanPub_, unknownScanPub_;
    bool publishClassifiedScans_;

    std::string failureProbabilityMarkerName_, markerFrame_;
    ros::Publisher failureProbabilityMarkerPub_;
    bool publishFailureProbabilityMarker_;

    // parametsrs
    double maxResidualError_;
    double NDMean_, NDVar_, NDNormConst_, EDLambda_;
    int minValidResidualErrorsNum_, maxResidualErrorsNum_;
    int maxLPBComputationNum_;
    int samplingNum_;
    double residualErrorReso_;
    double misalignmentRatioThreshold_, unknownRatioThreshold_;
    std::vector<double> transitionProbMat_;

    sensor_msgs::LaserScan residualErrors_;
    std::vector<double> usedResidualErrors_;
    std::vector<int> usedScanIndices_;
    bool canUpdateResidualErrors_, gotResidualErrors_;
    double failureDetectionHz_;

    // results
    std::vector<std::vector<double>> measurementClassProbabilities_;
    double failureProbability_;
    ros::Time failureProbabilityStamp_;

public:
    MRFFD():
        nh_("~"),
        residualErrorsName_("/residual_errors"),
        failureProbName_("/failure_probability"),
        alignedScanName_("/aligned_scan_mrf"),
        misalignedScanName_("/misaligned_scan_mrf"),
        unknownScanName_("/unknown_scan_mrf"),
        publishClassifiedScans_(true),
        failureProbabilityMarkerName_("/failure_probability_marker"),
        publishFailureProbabilityMarker_(true),
        markerFrame_("base_link"),
        NDMean_(0.0),
        NDVar_(0.04),
        EDLambda_(4.0),
        maxResidualError_(1.0),
        residualErrorReso_(0.05),
        minValidResidualErrorsNum_(10),
        maxResidualErrorsNum_(200),
        maxLPBComputationNum_(1000),
        samplingNum_(1000),
        misalignmentRatioThreshold_(0.1),
        unknownRatioThreshold_(0.7),
        transitionProbMat_({0.8, 0.0, 0.2, 0.0, 0.8, 0.2, 0.333333, 0.333333, 0.333333}),
        canUpdateResidualErrors_(true),
        gotResidualErrors_(false),
        failureDetectionHz_(10.0)
    {
        // input and output message names
        nh_.param("residual_errors_name", residualErrorsName_, residualErrorsName_);
        nh_.param("failure_probability_name", failureProbName_, failureProbName_);
        nh_.param("publish_classified_scans", publishClassifiedScans_, publishClassifiedScans_);
        nh_.param("aligned_scan_mrf", alignedScanName_, alignedScanName_);
        nh_.param("misaligned_scan_mrf", misalignedScanName_, misalignedScanName_);
        nh_.param("unknown_scan_mrf", unknownScanName_, unknownScanName_);
        nh_.param("failure_probability_marker_name", failureProbabilityMarkerName_, failureProbabilityMarkerName_);
        nh_.param("publish_failure_probability_marker", publishFailureProbabilityMarker_, publishFailureProbabilityMarker_);
        nh_.param("marker_frame", markerFrame_, markerFrame_);

        // parameters
        nh_.param("normal_distribution_mean", NDMean_, NDMean_);
        nh_.param("normal_distribution_var", NDVar_, NDVar_);
        nh_.param("exponential_distribution_lambda", EDLambda_, EDLambda_);
        nh_.param("max_residual_error", maxResidualError_, maxResidualError_);
        nh_.param("residual_error_resolution", residualErrorReso_, residualErrorReso_);
        nh_.param("min_valid_residual_errors_num", minValidResidualErrorsNum_, minValidResidualErrorsNum_);
        nh_.param("max_residual_errors_num", maxResidualErrorsNum_, maxResidualErrorsNum_);
        nh_.param("max_lpb_computation_num", maxLPBComputationNum_, maxLPBComputationNum_);
        nh_.param("sampling_num", samplingNum_, samplingNum_);
        nh_.param("misalignment_ratio_threshold", misalignmentRatioThreshold_, misalignmentRatioThreshold_);
        nh_.param("unknown_ratio_threshold", unknownRatioThreshold_, unknownRatioThreshold_);
        nh_.param("transition_probability_matrix", transitionProbMat_, transitionProbMat_);

        // other parameters
        nh_.param("failure_detection_hz", failureDetectionHz_, failureDetectionHz_);

        // ros subscriber and publisher
        residualErrorsSub_ = nh_.subscribe(residualErrorsName_, 1, &MRFFD::residualErrorsCB, this);
        failureProbPub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(failureProbName_, 1);
        if (publishClassifiedScans_) {
            alignedScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(alignedScanName_, 1);
            misalignedScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(misalignedScanName_, 1);
            unknownScanPub_ = nh_.advertise<sensor_msgs::LaserScan>(unknownScanName_, 1);
        }
        if (publishFailureProbabilityMarker_)
            failureProbabilityMarkerPub_ = nh_.advertise<visualization_msgs::Marker>(failureProbabilityMarkerName_, 1);

        // fixed parameters
        NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);

        // wait for getting the residual errors
        ros::Rate loopRate(failureDetectionHz_);
        int residualErrorsFailedCnt = 0;
        while (!gotResidualErrors_) {
            ros::spinOnce();
            residualErrorsFailedCnt++;
            if (residualErrorsFailedCnt >= 30) {
                ROS_ERROR("Cannot get residual errors."
                    " Did you publish the residual errors?"
                    " The expected topic name is %s", residualErrorsName_.c_str());
            }
            loopRate.sleep();
        }

        ROS_INFO("MRF failure detector is ready to perform.");
    }

    ~MRFFD() {};

    inline void setMaxResidualError(double maxResidualError) { maxResidualError_ = maxResidualError; }
    inline void setNDMean(double NDMean) { NDMean_ = NDMean; }
    inline void setNDVariance(double NDVar) { NDVar_ = NDVar, NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_); }
    inline void setEDLambda(double EDLambda) { EDLambda_ = EDLambda; }
    inline void setResidualErrorReso(double residualErrorReso) { residualErrorReso_ = residualErrorReso; }
    inline void setMinValidResidualErrorNum(int minValidResidualErrorNum) { minValidResidualErrorsNum_ = minValidResidualErrorNum; }
    inline void setMaxLPBComputationNum(int maxLPBComputationNum) { maxLPBComputationNum_ = maxLPBComputationNum; }
    inline void setSamplingNum(int samplingNum) { samplingNum_ = samplingNum; }
    inline void setMisalignmentRatioThreshold(double misalignmentRatioThreshold) { misalignmentRatioThreshold_ = misalignmentRatioThreshold; }
    inline void setTransitionProbMat(std::vector<double> transitionProbMat) { transitionProbMat_ = transitionProbMat; }
    inline void setCanUpdateResidualErrors(bool canUpdateResidualErrors) { canUpdateResidualErrors_ = canUpdateResidualErrors; }

    inline double getFailureProbability(void) { return failureProbability_; }
    inline double getMeasurementClassProbabilities(int errorIndex, int measurementClass) { return measurementClassProbabilities_[errorIndex][measurementClass]; }
    inline std::vector<double> getMeasurementClassProbabilities(int errorIndex) { return measurementClassProbabilities_[errorIndex]; }
    inline double getFailureDetectionHz(void) { return failureDetectionHz_; }

    void predictFailureProbability(void) {
        std::vector<double> validResidualErrors;
        std::vector<int> validScanIndices;
        for (int i = 0; i < (int)residualErrors_.intensities.size(); ++i) {
            double e = residualErrors_.intensities[i];
            if (0.0 <= e && e <= maxResidualError_) {
                validResidualErrors.push_back(e);
                validScanIndices.push_back(i);
            }
        }

        int validResidualErrorsSize = (int)validResidualErrors.size();
        if (validResidualErrorsSize <= minValidResidualErrorsNum_) {
            std::cerr << "WARNING: Number of validResidualErrors is less than the expected threshold number." <<
                " The threshold is " << minValidResidualErrorsNum_ <<
                ", but the number of validResidualErrors " << validResidualErrorsSize << "." << std::endl;
            failureProbability_ = -1.0;
            return;
        } else if (validResidualErrorsSize <= maxResidualErrorsNum_) {
            usedResidualErrors_ = validResidualErrors;
            usedScanIndices_ = validScanIndices;
        } else {
            usedResidualErrors_.resize(maxResidualErrorsNum_);
            usedScanIndices_.resize(maxResidualErrorsNum_);
            for (int i = 0; i < maxResidualErrorsNum_; ++i) {
                int idx = rand() % (int)validResidualErrors.size();
                usedResidualErrors_[i] = validResidualErrors[idx];
                usedScanIndices_[i] = validScanIndices[idx];
                validResidualErrors.erase(validResidualErrors.begin() + idx);
                validScanIndices.erase(validScanIndices.begin() + idx);
            }
        }

        std::vector<std::vector<double>> likelihoodVectors = getLikelihoodVectors(usedResidualErrors_);
        std::vector<std::vector<double>> measurementClassProbabilities = estimateMeasurementClassProbabilities(likelihoodVectors);
        setAllMeasurementClassProbabilities(usedResidualErrors_, measurementClassProbabilities);
        failureProbability_ = predictFailureProbabilityBySampling(measurementClassProbabilities);
    }

    void publishROSMessages(void) {
        geometry_msgs::Vector3Stamped failureProbability;
        failureProbability.header.stamp = residualErrors_.header.stamp;
        failureProbability.vector.x = failureProbability_;
        failureProbPub_.publish(failureProbability);

        if (publishClassifiedScans_) {
            std::vector<int> residualErrorClasses = getResidualErrorClasses();
            sensor_msgs::LaserScan alignedScan, misalignedScan, unknownScan;
            alignedScan.header = misalignedScan.header = unknownScan.header = residualErrors_.header;
            alignedScan.range_min = misalignedScan.range_min = unknownScan.range_min = residualErrors_.range_min;
            alignedScan.range_max = misalignedScan.range_max = unknownScan.range_max = residualErrors_.range_max;
            alignedScan.angle_min = misalignedScan.angle_min = unknownScan.angle_min = residualErrors_.angle_min;
            alignedScan.angle_max = misalignedScan.angle_max = unknownScan.angle_max = residualErrors_.angle_max;
            alignedScan.angle_increment = misalignedScan.angle_increment = unknownScan.angle_increment = residualErrors_.angle_increment;
            alignedScan.time_increment = misalignedScan.time_increment = unknownScan.time_increment = residualErrors_.time_increment;
            alignedScan.scan_time = misalignedScan.scan_time = unknownScan.scan_time = residualErrors_.scan_time;
            int size = (int)residualErrors_.ranges.size();
            alignedScan.ranges.resize(size);
            misalignedScan.ranges.resize(size);
            unknownScan.ranges.resize(size);
            alignedScan.intensities.resize(size);
            misalignedScan.intensities.resize(size);
            unknownScan.intensities.resize(size);
            for (int i = 0; i < (int)usedResidualErrors_.size(); ++i) {
                int idx = usedScanIndices_[i];
                if (residualErrorClasses[i] == ALIGNED)
                    alignedScan.ranges[idx] = residualErrors_.ranges[idx];
                else if (residualErrorClasses[i] == MISALIGNED)
                    misalignedScan.ranges[idx] = residualErrors_.ranges[idx];
                else
                    unknownScan.ranges[idx] = residualErrors_.ranges[idx];
            }
            alignedScanPub_.publish(alignedScan);
            misalignedScanPub_.publish(misalignedScan);
            unknownScanPub_.publish(unknownScan);
        }

        if (publishFailureProbabilityMarker_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = markerFrame_;
            marker.header.stamp = residualErrors_.header.stamp;
            marker.ns = "fp_marker_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = 0.0;
            marker.pose.position.y = -3.0;
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.0;
            marker.scale.y = 0.0;
            marker.scale.z = 2.0;
            marker.text = "Failure Probability: " + std::to_string((int)(failureProbability_ * 100.0)) + " %";
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            if (failureProbability_ > 0.1)
                marker.color.r = marker.color.g = 0.0;
            failureProbabilityMarkerPub_.publish(marker);
        }
    }

    void printFailureProbability(void) {
        std::cout << "Failure probability = " << failureProbability_ * 100.0 << " [%]" << std::endl;
    }

private:
    void residualErrorsCB(const sensor_msgs::LaserScan::ConstPtr &msg) {
        if (canUpdateResidualErrors_)
            residualErrors_ = *msg;
        if (!gotResidualErrors_)
            gotResidualErrors_ = true;
    }

    inline double calculateNormalDistribution(double e) {
        return (0.95 * (2.0 * NDNormConst_ * exp(-((e - NDMean_) * (e - NDMean_)) / (2.0 * NDVar_))) + 0.05 * (1.0 / maxResidualError_)) * residualErrorReso_;
    }

    inline double calculateExponentialDistribution(double e) {
        return (0.95 * (1.0 / (1.0 - exp(-EDLambda_ * maxResidualError_))) * EDLambda_ * exp(-EDLambda_ * e) + 0.05 * (1.0 / maxResidualError_)) * residualErrorReso_;
    }

    inline double calculateUniformDistribution(void) {
        return (1.0 / maxResidualError_) * residualErrorReso_;
    }

    inline double getSumOfVecotr(std::vector<double> vector) {
        double sum = 0.0;
        for (int i = 0; i < (int)vector.size(); i++)
            sum += vector[i];
        return sum;
    }

    inline std::vector<double> getHadamardProduct(std::vector<double> vector1, std::vector<double> vector2) {
        for (int i = 0; i < (int)vector1.size(); i++)
            vector1[i] *= vector2[i];
        return vector1;
    }

    inline std::vector<double> normalizeVector(std::vector<double> vector) {
        double sum = getSumOfVecotr(vector);
        for (int i = 0; i < (int)vector.size(); i++)
            vector[i] /= sum;
        return vector;
    }

    inline double getEuclideanNormOfDiffVectors(std::vector<double> vector1, std::vector<double> vector2) {
        double sum = 0.0;
        for (int i = 0; i < (int)vector1.size(); i++) {
            double diff = vector1[i] - vector2[i];
            sum += diff * diff;
        }
        return sqrt(sum);
    }

    inline std::vector<double> calculateTransitionMessage(std::vector<double> probs) {
        std::vector<double> message(3);
        std::vector<double> tm = transitionProbMat_;
        message[ALIGNED] = tm[ALIGNED] * probs[ALIGNED] + tm[MISALIGNED] * probs[MISALIGNED] + tm[UNKNOWN] * probs[UNKNOWN];
        message[MISALIGNED] = tm[ALIGNED + 3] * probs[ALIGNED] + tm[MISALIGNED + 3] * probs[MISALIGNED] + tm[UNKNOWN + 3] * probs[UNKNOWN];
        message[UNKNOWN] = tm[ALIGNED + 6] * probs[ALIGNED] + tm[MISALIGNED + 6] * probs[MISALIGNED] + tm[UNKNOWN + 6] * probs[UNKNOWN];
        return message;
    }

    std::vector<std::vector<double>> getLikelihoodVectors(std::vector<double> validResidualErrors) {
        std::vector<std::vector<double>> likelihoodVectors((int)validResidualErrors.size());
        double pud = calculateUniformDistribution();
        for (int i = 0; i < (int)likelihoodVectors.size(); i++) {
            likelihoodVectors[i].resize(3);
            likelihoodVectors[i][ALIGNED] = calculateNormalDistribution(validResidualErrors[i]);
            likelihoodVectors[i][MISALIGNED] = calculateExponentialDistribution(validResidualErrors[i]);
            likelihoodVectors[i][UNKNOWN] = pud;
            likelihoodVectors[i] = normalizeVector(likelihoodVectors[i]);
        }
        return likelihoodVectors;
    }

    std::vector<std::vector<double>> estimateMeasurementClassProbabilities(std::vector<std::vector<double>> likelihoodVectors) {
        std::vector<std::vector<double>> measurementClassProbabilities = likelihoodVectors;
        for (int i = 0; i < (int)measurementClassProbabilities.size(); i++) {
            for (int j = 0; j < (int)measurementClassProbabilities.size(); j++) {
                if (i == j)
                    continue;
                std::vector<double> message = calculateTransitionMessage(likelihoodVectors[j]);
                measurementClassProbabilities[i] = getHadamardProduct(measurementClassProbabilities[i], message);
                measurementClassProbabilities[i] = normalizeVector(measurementClassProbabilities[i]);
            }
            measurementClassProbabilities[i] = normalizeVector(measurementClassProbabilities[i]);
        }

        double variation = 0.0;
        int idx1 = rand() % (int)measurementClassProbabilities.size();
        std::vector<double> message(3);
        message = likelihoodVectors[idx1];
        int checkStep = maxLPBComputationNum_ / 20;
        for (int i = 0; i < maxLPBComputationNum_; i++) {
            int idx2 = rand() % (int)measurementClassProbabilities.size();
            int cnt = 0;
            for (;;) {
                if (idx2 != idx1)
                    break;
                idx2 = rand() % (int)measurementClassProbabilities.size();
                cnt++;
                if (cnt >= 10)
                    break;
            }
            message = calculateTransitionMessage(message);
            message = getHadamardProduct(likelihoodVectors[idx2], message);
            std::vector<double> measurementClassProbabilitiesPrev = measurementClassProbabilities[idx2];
            measurementClassProbabilities[idx2] = getHadamardProduct(measurementClassProbabilities[idx2], message);
            measurementClassProbabilities[idx2] = normalizeVector(measurementClassProbabilities[idx2]);
            double diffNorm = getEuclideanNormOfDiffVectors(measurementClassProbabilities[idx2], measurementClassProbabilitiesPrev);
            variation += diffNorm;
            if (i >= checkStep && i % checkStep == 0 && variation < 10e-6)
                break;
            else if (i >= checkStep && i % checkStep == 0)
                variation = 0.0;
            message = measurementClassProbabilities[idx2];
            idx1 = idx2;
        }
        return measurementClassProbabilities;
    }

    double predictFailureProbabilityBySampling(std::vector<std::vector<double>> measurementClassProbabilities) {
        int failureCnt = 0;
        for (int i = 0; i < samplingNum_; i++) {
            int misalignedNum = 0, validMeasurementNum = 0;
            int measurementNum = (int)measurementClassProbabilities.size();
            for (int j = 0; j < measurementNum; j++) {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                double validProb = measurementClassProbabilities[j][ALIGNED] + measurementClassProbabilities[j][MISALIGNED];
                if (darts > validProb)
                    continue;
                validMeasurementNum++;
                if (darts > measurementClassProbabilities[j][ALIGNED])
                    misalignedNum++;
            }
            double misalignmentRatio = (double)misalignedNum / (double)validMeasurementNum;
            double unknownRatio = (double)(measurementNum - validMeasurementNum) / (double)measurementNum;
            if (misalignmentRatio >= misalignmentRatioThreshold_
                || unknownRatio >= unknownRatioThreshold_)
                failureCnt++;
        }
        double p = (double)failureCnt / (double)samplingNum_;
        return p;
    }

    void setAllMeasurementClassProbabilities(std::vector<double> residualErrors, std::vector<std::vector<double>> measurementClassProbabilities) {
        measurementClassProbabilities_.resize((int)residualErrors.size());
        int idx = 0;
        for (int i = 0; i < (int)measurementClassProbabilities_.size(); i++) {
            measurementClassProbabilities_[i].resize(3);
            if (0.0 <= residualErrors[i] && residualErrors[i] <= maxResidualError_) {
                measurementClassProbabilities_[i] = measurementClassProbabilities[idx];
                idx++;
            } else {
                measurementClassProbabilities_[i][ALIGNED] = 0.00005;
                measurementClassProbabilities_[i][MISALIGNED] = 0.00005;
                measurementClassProbabilities_[i][UNKNOWN] = 0.9999;
            }
        }
    }

    std::vector<int> getResidualErrorClasses(void) {
        int size = (int)measurementClassProbabilities_.size();
        std::vector<int> residualErrorClasses(size);
        for (int i = 0; i < size; i++) {
            double alignedProb = measurementClassProbabilities_[i][ALIGNED];
            double misalignedProb = measurementClassProbabilities_[i][MISALIGNED];
            double unknownProb = measurementClassProbabilities_[i][UNKNOWN];
            if (alignedProb > misalignedProb && alignedProb > unknownProb)
                residualErrorClasses[i] = ALIGNED;
            else if (misalignedProb > alignedProb && misalignedProb > unknownProb)
                residualErrorClasses[i] = MISALIGNED;
            else
                residualErrorClasses[i] = UNKNOWN;
        }
        return residualErrorClasses;
    }
}; // class MRFFD

} // namespace als_ros

#endif // __MRF_FAILURE_DETECTOR_H__

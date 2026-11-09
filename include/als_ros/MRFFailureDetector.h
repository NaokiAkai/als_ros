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

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>
#include <vector>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::placeholders;

namespace als_ros
{

enum MeasurementClass
{
    ALIGNED = 0,
    MISALIGNED = 1,
    UNKNOWN = 2
};

class MRFFD : public rclcpp::Node
{
  private:
    // ros subscribers and publishers
    std::string residualErrorsName_, occlusionScoreName_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr residualErrorsSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr occlusionScoreSub_;

    std::string failureProbName_, alignedScanName_, misalignedScanName_, unknownScanName_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr failureProbPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr alignedScanPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr misalignedScanPub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unknownScanPub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr relocalizationToggleSrv_;

    bool relocalizationEnabled_;
    bool publishClassifiedScans_;

    std::string failureProbabilityMarkerName_, markerFrame_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failureProbabilityMarkerPub_;
    bool publishFailureProbabilityMarker_;

    // parameters
    double maxResidualError_;
    double NDMean_, NDVar_, NDNormConst_, EDLambda_;
    int minValidResidualErrorsNum_, maxResidualErrorsNum_;
    int maxLPBComputationNum_;
    int samplingNum_;
    double residualErrorReso_;
    double misalignmentRatioThreshold_, unknownRatioThreshold_;
    std::vector<double> transitionProbMat_;

    sensor_msgs::msg::LaserScan residualErrors_;
    std::vector<double> usedResidualErrors_;
    std::vector<int> usedScanIndices_;
    bool canUpdateResidualErrors_, gotResidualErrors_;
    double failureDetectionHz_;
    double occlusionScore_;

    // results
    std::vector<std::vector<double>> measurementClassProbabilities_;
    double failureProbability_;
    bool inFailure_;
    double failureProbabilityTreshold_;
    double failureDurationThreshold_;
    double occlusionScoreThreshold_;
    double relocFailureDurationThreshold_;
    double relocOcclusionScoreThreshold_;
    rclcpp::Time failureTimestamp_;

  public:
    MRFFD()
        : Node("mrffd"), residualErrorsName_("residual_errors"), occlusionScoreName_("occlusion_score"),
          failureProbName_("localization_failure"), alignedScanName_("aligned_scan_mrf"),
          misalignedScanName_("misaligned_scan_mrf"), unknownScanName_("unknown_scan_mrf"),
          relocalizationEnabled_(false), publishClassifiedScans_(true),
          failureProbabilityMarkerName_("failure_probability_marker"), publishFailureProbabilityMarker_(true),
          markerFrame_("base_link"), NDMean_(0.0), NDVar_(0.04), EDLambda_(4.0), maxResidualError_(1.0),
          residualErrorReso_(0.05), minValidResidualErrorsNum_(10), maxResidualErrorsNum_(200),
          maxLPBComputationNum_(1000), samplingNum_(1000), misalignmentRatioThreshold_(0.1),
          unknownRatioThreshold_(0.7), transitionProbMat_({0.8, 0.0, 0.2, 0.0, 0.8, 0.2, 0.333333, 0.333333, 0.333333}),
          canUpdateResidualErrors_(true), gotResidualErrors_(false), failureDetectionHz_(10.0), occlusionScore_(0.0),
          inFailure_(false), failureProbabilityTreshold_(0.5), failureDurationThreshold_(5.0),
          occlusionScoreThreshold_(0.2), relocFailureDurationThreshold_(0.0), relocOcclusionScoreThreshold_(0.1)
    {
        // input and output message names
        declare_parameter("residual_errors_name", residualErrorsName_);
        declare_parameter("occlusion_score_name", occlusionScoreName_);
        declare_parameter("failure_probability_name", failureProbName_);
        declare_parameter("publish_classified_scans", publishClassifiedScans_);
        declare_parameter("aligned_scan_mrf", alignedScanName_);
        declare_parameter("misaligned_scan_mrf", misalignedScanName_);
        declare_parameter("unknown_scan_mrf", unknownScanName_);
        declare_parameter("failure_probability_marker_name", failureProbabilityMarkerName_);
        declare_parameter("publish_failure_probability_marker", publishFailureProbabilityMarker_);
        declare_parameter("marker_frame", markerFrame_);

        get_parameter("residual_errors_name", residualErrorsName_);
        get_parameter("occlusion_score_name", occlusionScoreName_);
        get_parameter("failure_probability_name", failureProbName_);
        get_parameter("publish_classified_scans", publishClassifiedScans_);
        get_parameter("aligned_scan_mrf", alignedScanName_);
        get_parameter("misaligned_scan_mrf", misalignedScanName_);
        get_parameter("unknown_scan_mrf", unknownScanName_);
        get_parameter("failure_probability_marker_name", failureProbabilityMarkerName_);
        get_parameter("publish_failure_probability_marker", publishFailureProbabilityMarker_);
        get_parameter("marker_frame", markerFrame_);

        // parameters
        declare_parameter("normal_distribution_mean", NDMean_);
        declare_parameter("normal_distribution_var", NDVar_);
        declare_parameter("exponential_distribution_lambda", EDLambda_);
        declare_parameter("max_residual_error", maxResidualError_);
        declare_parameter("residual_error_resolution", residualErrorReso_);
        declare_parameter("min_valid_residual_errors_num", minValidResidualErrorsNum_);
        declare_parameter("max_residual_errors_num", maxResidualErrorsNum_);
        declare_parameter("max_lpb_computation_num", maxLPBComputationNum_);
        declare_parameter("sampling_num", samplingNum_);
        declare_parameter("misalignment_ratio_threshold", misalignmentRatioThreshold_);
        declare_parameter("unknown_ratio_threshold", unknownRatioThreshold_);
        declare_parameter("transition_probability_matrix", transitionProbMat_);

        get_parameter("normal_distribution_mean", NDMean_);
        get_parameter("normal_distribution_var", NDVar_);
        get_parameter("exponential_distribution_lambda", EDLambda_);
        get_parameter("max_residual_error", maxResidualError_);
        get_parameter("residual_error_resolution", residualErrorReso_);
        get_parameter("min_valid_residual_errors_num", minValidResidualErrorsNum_);
        get_parameter("max_residual_errors_num", maxResidualErrorsNum_);
        get_parameter("max_lpb_computation_num", maxLPBComputationNum_);
        get_parameter("sampling_num", samplingNum_);
        get_parameter("misalignment_ratio_threshold", misalignmentRatioThreshold_);
        get_parameter("unknown_ratio_threshold", unknownRatioThreshold_);
        get_parameter("transition_probability_matrix", transitionProbMat_);

        // other parameters
        declare_parameter("failure_detection_hz", failureDetectionHz_);
        declare_parameter("failure_probability_threshold", failureProbabilityTreshold_);
        declare_parameter("failure_duration_threshold", failureDurationThreshold_);
        declare_parameter("occlusion_score_threshold", occlusionScoreThreshold_);
        declare_parameter("reloc_failure_duration_threshold", relocFailureDurationThreshold_);
        declare_parameter("reloc_occlusion_score_threshold", relocOcclusionScoreThreshold_);

        get_parameter("failure_detection_hz", failureDetectionHz_);
        get_parameter("failure_probability_threshold", failureProbabilityTreshold_);
        get_parameter("failure_duration_threshold", failureDurationThreshold_);
        get_parameter("occlusion_score_threshold", occlusionScoreThreshold_);
        get_parameter("reloc_failure_duration_threshold", relocFailureDurationThreshold_);
        get_parameter("reloc_occlusion_score_threshold", relocOcclusionScoreThreshold_);

        // ros subscriber and publisher
        residualErrorsSub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            residualErrorsName_, 1, 
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                this->residualErrorsCB(msg);
            }
        );
        occlusionScoreSub_ = create_subscription<std_msgs::msg::Float32>(
            occlusionScoreName_, 1,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                this->occlusionScoreCB(msg);
            }
        );
        failureProbPub_ = create_publisher<std_msgs::msg::Bool>(failureProbName_, 1);
        if (publishClassifiedScans_)
        {
            alignedScanPub_ = create_publisher<sensor_msgs::msg::LaserScan>(alignedScanName_, 1);
            misalignedScanPub_ = create_publisher<sensor_msgs::msg::LaserScan>(misalignedScanName_, 1);
            unknownScanPub_ = create_publisher<sensor_msgs::msg::LaserScan>(unknownScanName_, 1);
        }
        if (publishFailureProbabilityMarker_)
            failureProbabilityMarkerPub_ =
                create_publisher<visualization_msgs::msg::Marker>(failureProbabilityMarkerName_, 1);

        relocalizationToggleSrv_ = this->create_service<std_srvs::srv::SetBool>(
            "mrf/toggle_relocalization_mode",
            [this](const std::shared_ptr<rmw_request_id_t> /*req_header*/,
               const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
            this->relocalizationEnabled_ = req->data;
            resp->success = true;
            resp->message = this->relocalizationEnabled_ ? "relocalization enabled" : "relocalization disabled";
            }
        );

        // fixed parameters
        NDVar_ = NDVar_ * NDVar_;
        NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);

        // wait for getting the residual errors
        rclcpp::Rate loopRate(failureDetectionHz_);
        int residualErrorsFailedCnt = 0;
        while (!gotResidualErrors_)
        {
            // TODO: check if this works, if not switch to wait_for_message
            rclcpp::spin_some(get_node_base_interface());
            residualErrorsFailedCnt++;
            if (residualErrorsFailedCnt >= 30)
            {
                RCLCPP_ERROR(get_logger(),
                             "Cannot get residual errors."
                             " Did you publish the residual errors?"
                             " The expected topic name is %s",
                             residualErrorsName_.c_str());
            }
            loopRate.sleep();
        }

        RCLCPP_INFO(get_logger(), "MRF failure detector is ready to perform.");
    }

    ~MRFFD(){};

    inline void setMaxResidualError(double maxResidualError)
    {
        maxResidualError_ = maxResidualError;
    }
    inline void setNDMean(double NDMean)
    {
        NDMean_ = NDMean;
    }
    inline void setNDVariance(double NDVar)
    {
        NDVar_ = NDVar, NDNormConst_ = 1.0 / sqrt(2.0 * M_PI * NDVar_);
    }
    inline void setEDLambda(double EDLambda)
    {
        EDLambda_ = EDLambda;
    }
    inline void setResidualErrorReso(double residualErrorReso)
    {
        residualErrorReso_ = residualErrorReso;
    }
    inline void setMinValidResidualErrorNum(int minValidResidualErrorNum)
    {
        minValidResidualErrorsNum_ = minValidResidualErrorNum;
    }
    inline void setMaxLPBComputationNum(int maxLPBComputationNum)
    {
        maxLPBComputationNum_ = maxLPBComputationNum;
    }
    inline void setSamplingNum(int samplingNum)
    {
        samplingNum_ = samplingNum;
    }
    inline void setMisalignmentRatioThreshold(double misalignmentRatioThreshold)
    {
        misalignmentRatioThreshold_ = misalignmentRatioThreshold;
    }
    inline void setTransitionProbMat(std::vector<double> transitionProbMat)
    {
        transitionProbMat_ = transitionProbMat;
    }
    inline void setCanUpdateResidualErrors(bool canUpdateResidualErrors)
    {
        canUpdateResidualErrors_ = canUpdateResidualErrors;
    }

    inline double getFailureProbability(void)
    {
        return failureProbability_;
    }
    inline double getMeasurementClassProbabilities(int errorIndex, int measurementClass)
    {
        return measurementClassProbabilities_[errorIndex][measurementClass];
    }
    inline std::vector<double> getMeasurementClassProbabilities(int errorIndex)
    {
        return measurementClassProbabilities_[errorIndex];
    }
    inline double getFailureDetectionHz(void)
    {
        return failureDetectionHz_;
    }

    void predictFailureProbability(void)
    {
        std::vector<double> validResidualErrors;
        std::vector<int> validScanIndices;
        int offMapReturnsNum = 0;
        for (int i = 0; i < (int)residualErrors_.intensities.size(); ++i)
        {
            double e = residualErrors_.intensities[i];
            if (0.0 <= e && e <= maxResidualError_)
            {
                validResidualErrors.push_back(e);
                validScanIndices.push_back(i);
            }
            else if (e < 0.0 && isLidarReturn(residualErrors_.ranges[i]))
            {
                offMapReturnsNum++;
            }
        }

        int validResidualErrorsSize = (int)validResidualErrors.size();
        if (validResidualErrorsSize <= minValidResidualErrorsNum_)
        {
            std::cerr << "WARNING: Number of validResidualErrors is less than the expected threshold number."
                      << " The threshold is " << minValidResidualErrorsNum_
                      << ", but the number of validResidualErrors " << validResidualErrorsSize << ", with "
                      << offMapReturnsNum << " off-map returns." << std::endl;
            failureProbability_ = offMapReturnsNum > minValidResidualErrorsNum_ ? 1.0 : -1.0;
            return;
        }
        else if (validResidualErrorsSize <= maxResidualErrorsNum_)
        {
            usedResidualErrors_ = validResidualErrors;
            usedScanIndices_ = validScanIndices;
        }
        else
        {
            usedResidualErrors_.resize(maxResidualErrorsNum_);
            usedScanIndices_.resize(maxResidualErrorsNum_);
            for (int i = 0; i < maxResidualErrorsNum_; ++i)
            {
                // downsample scan evenly
                int idx = (int)(i * ((double)validResidualErrorsSize / (double)maxResidualErrorsNum_));
                usedResidualErrors_[i] = validResidualErrors[idx];
                usedScanIndices_[i] = validScanIndices[idx];
            }
        }

        std::vector<std::vector<double>> likelihoodVectors = getLikelihoodVectors(usedResidualErrors_);
        std::vector<std::vector<double>> measurementClassProbabilities =
            estimateMeasurementClassProbabilities(likelihoodVectors);
        setAllMeasurementClassProbabilities(usedResidualErrors_, measurementClassProbabilities);
        failureProbability_ = predictFailureProbabilityBySampling(measurementClassProbabilities);
    }

    void publishROSMessages(void)
    {
        std_msgs::msg::Bool failure;
        failure.data = false;
        double occlusionScoreThreshold = relocalizationEnabled_
                                     ? relocOcclusionScoreThreshold_
                                     : occlusionScoreThreshold_;
        double failureDurationThreshold = relocalizationEnabled_
                                     ? relocFailureDurationThreshold_
                                     : failureDurationThreshold_;
        if (failureProbability_ <= failureProbabilityTreshold_ || occlusionScore_ <= occlusionScoreThreshold)
        {
            inFailure_ = false;
        }
        else if (!inFailure_)
        {
            inFailure_ = true;
            failureTimestamp_ = this->now();
        }
        else if (this->now() - failureTimestamp_ > rclcpp::Duration::from_seconds(failureDurationThreshold))
        {
            failure.data = true;
        }
        failureProbPub_->publish(failure);

        if (publishClassifiedScans_)
        {
            std::vector<int> residualErrorClasses = getResidualErrorClasses();
            sensor_msgs::msg::LaserScan alignedScan, misalignedScan, unknownScan;
            alignedScan.header = misalignedScan.header = unknownScan.header = residualErrors_.header;
            alignedScan.range_min = misalignedScan.range_min = unknownScan.range_min = residualErrors_.range_min;
            alignedScan.range_max = misalignedScan.range_max = unknownScan.range_max = residualErrors_.range_max;
            alignedScan.angle_min = misalignedScan.angle_min = unknownScan.angle_min = residualErrors_.angle_min;
            alignedScan.angle_max = misalignedScan.angle_max = unknownScan.angle_max = residualErrors_.angle_max;
            alignedScan.angle_increment = misalignedScan.angle_increment = unknownScan.angle_increment =
                residualErrors_.angle_increment;
            alignedScan.time_increment = misalignedScan.time_increment = unknownScan.time_increment =
                residualErrors_.time_increment;
            alignedScan.scan_time = misalignedScan.scan_time = unknownScan.scan_time = residualErrors_.scan_time;
            int size = (int)residualErrors_.ranges.size();
            alignedScan.ranges.resize(size);
            misalignedScan.ranges.resize(size);
            unknownScan.ranges.resize(size);
            alignedScan.intensities.resize(size);
            misalignedScan.intensities.resize(size);
            unknownScan.intensities.resize(size);
            for (int i = 0; i < (int)usedResidualErrors_.size(); ++i)
            {
                int idx = usedScanIndices_[i];
                if (residualErrorClasses[i] == ALIGNED)
                    alignedScan.ranges[idx] = residualErrors_.ranges[idx];
                else if (residualErrorClasses[i] == MISALIGNED)
                    misalignedScan.ranges[idx] = residualErrors_.ranges[idx];
                else
                    unknownScan.ranges[idx] = residualErrors_.ranges[idx];
            }
            alignedScanPub_->publish(alignedScan);
            misalignedScanPub_->publish(misalignedScan);
            unknownScanPub_->publish(unknownScan);
        }

        if (publishFailureProbabilityMarker_)
        {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = markerFrame_;
            marker.header.stamp = residualErrors_.header.stamp;
            marker.ns = "fp_marker_namespace";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::msg::Marker::ADD;
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
            failureProbabilityMarkerPub_->publish(marker);
        }
    }

    void printFailureProbability(void)
    {
        std::cout << "Failure probability = " << failureProbability_ * 100.0 << " [%]" << std::endl;
    }

  private:
    void residualErrorsCB(const sensor_msgs::msg::LaserScan::SharedPtr &msg)
    {
        if (canUpdateResidualErrors_)
            residualErrors_ = *msg;
        if (!gotResidualErrors_)
            gotResidualErrors_ = true;
    }
    void occlusionScoreCB(const std_msgs::msg::Float32::SharedPtr &msg)
    {
        occlusionScore_ = msg->data;
    }

    inline bool isLidarReturn(float range)
    {
        return residualErrors_.range_min < range && range < residualErrors_.range_max;
    }

    inline double calculateNormalDistribution(double e)
    {
        return (0.95 * (2.0 * NDNormConst_ * exp(-((e - NDMean_) * (e - NDMean_)) / (2.0 * NDVar_))) +
                0.05 * (1.0 / maxResidualError_)) *
               residualErrorReso_;
    }

    inline double calculateExponentialDistribution(double e)
    {
        return (0.95 * (1.0 / (1.0 - exp(-EDLambda_ * maxResidualError_))) * EDLambda_ * exp(-EDLambda_ * e) +
                0.05 * (1.0 / maxResidualError_)) *
               residualErrorReso_;
    }

    inline double calculateUniformDistribution(void)
    {
        return (1.0 / maxResidualError_) * residualErrorReso_;
    }

    inline double getSumOfVecotr(std::vector<double> vector)
    {
        double sum = 0.0;
        for (int i = 0; i < (int)vector.size(); i++)
            sum += vector[i];
        return sum;
    }

    inline std::vector<double> getHadamardProduct(std::vector<double> vector1, std::vector<double> vector2)
    {
        for (int i = 0; i < (int)vector1.size(); i++)
            vector1[i] *= vector2[i];
        return vector1;
    }

    inline std::vector<double> normalizeVector(std::vector<double> vector)
    {
        double sum = getSumOfVecotr(vector);
        for (int i = 0; i < (int)vector.size(); i++)
            vector[i] /= sum;
        return vector;
    }

    inline double getEuclideanNormOfDiffVectors(std::vector<double> vector1, std::vector<double> vector2)
    {
        double sum = 0.0;
        for (int i = 0; i < (int)vector1.size(); i++)
        {
            double diff = vector1[i] - vector2[i];
            sum += diff * diff;
        }
        return sqrt(sum);
    }

    inline std::vector<double> calculateTransitionMessage(std::vector<double> probs)
    {
        std::vector<double> message(3);
        std::vector<double> tm = transitionProbMat_;
        message[ALIGNED] =
            tm[ALIGNED] * probs[ALIGNED] + tm[MISALIGNED] * probs[MISALIGNED] + tm[UNKNOWN] * probs[UNKNOWN];
        message[MISALIGNED] = tm[ALIGNED + 3] * probs[ALIGNED] + tm[MISALIGNED + 3] * probs[MISALIGNED] +
                              tm[UNKNOWN + 3] * probs[UNKNOWN];
        message[UNKNOWN] = tm[ALIGNED + 6] * probs[ALIGNED] + tm[MISALIGNED + 6] * probs[MISALIGNED] +
                           tm[UNKNOWN + 6] * probs[UNKNOWN];
        return message;
    }

    std::vector<std::vector<double>> getLikelihoodVectors(std::vector<double> validResidualErrors)
    {
        std::vector<std::vector<double>> likelihoodVectors((int)validResidualErrors.size());
        double pud = calculateUniformDistribution();
        for (int i = 0; i < (int)likelihoodVectors.size(); i++)
        {
            likelihoodVectors[i].resize(3);
            likelihoodVectors[i][ALIGNED] = calculateNormalDistribution(validResidualErrors[i]);
            likelihoodVectors[i][MISALIGNED] = calculateExponentialDistribution(validResidualErrors[i]);
            likelihoodVectors[i][UNKNOWN] = pud;
            likelihoodVectors[i] = normalizeVector(likelihoodVectors[i]);
        }
        return likelihoodVectors;
    }

    std::vector<std::vector<double>> estimateMeasurementClassProbabilities(
        std::vector<std::vector<double>> likelihoodVectors)
    {
        std::vector<std::vector<double>> measurementClassProbabilities = likelihoodVectors;
        for (int i = 0; i < (int)measurementClassProbabilities.size(); i++)
        {
            for (int j = 0; j < (int)measurementClassProbabilities.size(); j++)
            {
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
        for (int i = 0; i < maxLPBComputationNum_; i++)
        {
            int idx2 = rand() % (int)measurementClassProbabilities.size();
            int cnt = 0;
            for (;;)
            {
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
            double diffNorm =
                getEuclideanNormOfDiffVectors(measurementClassProbabilities[idx2], measurementClassProbabilitiesPrev);
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

    double predictFailureProbabilityBySampling(std::vector<std::vector<double>> measurementClassProbabilities)
    {
        int failureCnt = 0;
        for (int i = 0; i < samplingNum_; i++)
        {
            int misalignedNum = 0, validMeasurementNum = 0;
            int measurementNum = (int)measurementClassProbabilities.size();
            for (int j = 0; j < measurementNum; j++)
            {
                double darts = (double)rand() / ((double)RAND_MAX + 1.0);
                double validProb =
                    measurementClassProbabilities[j][ALIGNED] + measurementClassProbabilities[j][MISALIGNED];
                if (darts > validProb)
                    continue;
                validMeasurementNum++;
                if (darts > measurementClassProbabilities[j][ALIGNED])
                    misalignedNum++;
            }
            double misalignmentRatio = (double)misalignedNum / (double)validMeasurementNum;
            double unknownRatio = (double)(measurementNum - validMeasurementNum) / (double)measurementNum;
            if (misalignmentRatio >= misalignmentRatioThreshold_ || unknownRatio >= unknownRatioThreshold_)
                failureCnt++;
        }
        double p = (double)failureCnt / (double)samplingNum_;
        return p;
    }

    void setAllMeasurementClassProbabilities(std::vector<double> residualErrors,
                                             std::vector<std::vector<double>> measurementClassProbabilities)
    {
        measurementClassProbabilities_.resize((int)residualErrors.size());
        int idx = 0;
        for (int i = 0; i < (int)measurementClassProbabilities_.size(); i++)
        {
            measurementClassProbabilities_[i].resize(3);
            if (0.0 <= residualErrors[i] && residualErrors[i] <= maxResidualError_)
            {
                measurementClassProbabilities_[i] = measurementClassProbabilities[idx];
                idx++;
            }
            else
            {
                measurementClassProbabilities_[i][ALIGNED] = 0.00005;
                measurementClassProbabilities_[i][MISALIGNED] = 0.00005;
                measurementClassProbabilities_[i][UNKNOWN] = 0.9999;
            }
        }
    }

    std::vector<int> getResidualErrorClasses(void)
    {
        int size = (int)measurementClassProbabilities_.size();
        std::vector<int> residualErrorClasses(size);
        for (int i = 0; i < size; i++)
        {
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

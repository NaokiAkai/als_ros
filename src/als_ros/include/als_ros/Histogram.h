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

#ifndef __HISTOGRAM_H__
#define __HISTOGRAM_H__

#include <iostream>
#include <vector>
#include <algorithm>

namespace als_ros {

class Histogram {
private:
    double binWidth_;
    double minValue_, maxValue_;
    int histogramSize_, valueNum_;
    std::vector<int> histogram_;
    std::vector<double> probability_;

    inline int val2bin(double val) {
        int bin = (int)((val - minValue_) / binWidth_);
        if (bin < 0 || histogramSize_ <= bin)
            bin = -1;
        return bin;
    }

    inline double bin2val(int bin) {
        return (double)bin * binWidth_ + minValue_;
    }

    void buildHistogram(std::vector<double> values) {
        histogramSize_ = (int)((maxValue_ - minValue_) / binWidth_) + 1;

        histogram_.resize(histogramSize_, 0);
        valueNum_ = 0;
        for (int i = 0; i < (int)values.size(); ++i) {
            int bin = val2bin(values[i]);
            if (bin >= 0) {
                histogram_[bin]++;
                valueNum_++;
            }
        }

        probability_.resize(histogramSize_, 0.0);
        for (int i = 0; i < (int)histogram_.size(); ++i)
            probability_[i] = (double)histogram_[i] / (double)valueNum_;
    }

public:
    Histogram(void) {}

    Histogram(std::vector<double> values, double binWidth):
        binWidth_(binWidth)
    {
        minValue_ = *std::min_element(values.begin(), values.end());
        maxValue_ = *std::max_element(values.begin(), values.end());
        buildHistogram(values);
    }

    Histogram(std::vector<double> values, double binWidth, double minValue, double maxValue):
        binWidth_(binWidth),
        minValue_(minValue),
        maxValue_(maxValue)
    {
        buildHistogram(values);
    }

    inline double getProbability(double value) {
        if (value >= maxValue_)
            value -= binWidth_;
        int bin = val2bin(value);
        if (bin >= 0)
            return probability_[bin];
        else
            return -1.0;
    }

    void smoothHistogram(void) {
        std::vector<double> vals((int)histogram_.size());
        double sum = 0.0;
        for (int i = 0; i < (int)histogram_.size(); ++i) {
            int i0 = i - 1;
            if (i0 < 0)
                i0 = 0;
            int i1 = i + 1;
            if (i1 >= (int)histogram_.size())
                i1 = (int)histogram_.size() - 1;
            double val = (histogram_[i0] + histogram_[i] + histogram_[i1]) / 3.0;
            vals[i] = val;
            sum += val;
        }
        for (int i = 0; i < (int)histogram_.size(); ++i)
            histogram_[i] = vals[i] / sum;
    }

    void printHistogram(void) {
        for (int i = 0; i < (int)histogram_.size(); ++i)
            printf("bin = %d: val = %lf, histogram[%d] = %d, probability[%d] = %lf\n", 
                i, bin2val(i), i, histogram_[i], i, probability_[i]);
    }
}; // class Histogram

} // namespace als_ros

#endif // __HISTOGRAM_H__

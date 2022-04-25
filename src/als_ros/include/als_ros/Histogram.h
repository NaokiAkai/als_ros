/****************************************************************************
 * ALSROS: Advanced 2D Localization Systems for ROS use
 * Copyright (C) 2022 Naoki Akai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at https://mozilla.org/MPL/2.0/.
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
//    static std::vector<double> probability_;
// MAEClassifierで6つヒストグラムを宣言すると、このprobability_を宣言するとnanになる
// ただし、MAEClassifierでヒストグラムの宣言を4つにすると落ちない
// staticをつけたらこの問題は回避できる。
// 現状はMAEClassifierで宣言するヒストグラムを4つにしている

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

    void printHistogram(void) {
        for (int i = 0; i < (int)histogram_.size(); ++i)
            printf("bin = %d: val = %lf, histogram[%d] = %d, probability[%d] = %lf\n", 
                i, bin2val(i), i, histogram_[i], i, probability_[i]);
    }
}; // class Histogram

} // namespace als_ros

#endif // __HISTOGRAM_H__

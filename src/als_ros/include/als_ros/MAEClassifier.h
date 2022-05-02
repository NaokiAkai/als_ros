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

#ifndef __MAE_CLASSIFIER_H__
#define __MAE_CLASSIFIER_H__

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <list>
#include <numeric>
#include <als_ros/Histogram.h>

namespace als_ros {

class MAEClassifier {
private:
    std::string classifiersDir_;
    double maxResidualError_;

    double failureThreshold_;
    double successMAEMean_, successMAEStd_;
    double failureMAEMean_, failureMAEStd_;

    int truePositiveNum_, falsePositiveNum_, trueNegativeNum_, falseNegativeNum_;
    double dTruePositive_, dFalsePositive_, dTrueNegative_, dFalseNegative_;

    double maeHistogramBinWidth_;
    Histogram positiveMAEHistogram_, negativeMAEHistogram_;
    Histogram truePositiveMAEHistogram_, trueNegativeMAEHistogram_;
    Histogram falsePositiveMAEHistogram_, falseNegativeMAEHistogram_;

    template <template<class T, class Allocator = std::allocator<T>> class Container> double getMean(Container<double> &x) {
        return std::accumulate(x.begin(), x.end(), 0.0) / x.size();
    }

    template <template<class T, class Allocator = std::allocator<T>> class Container> double getVar(Container<double> &x) {
        double size = x.size();
        double mean = getMean(x);
        return (std::inner_product(x.begin(), x.end(), x.begin(), 0.0) - mean * mean * size) / (size - 1.0);
    }

    template <template<class T, class Allocator = std::allocator<T>> class Container> double getStd(Container<double> &x) {
        return std::sqrt(getVar(x));
    }

    std::vector<double> readMAEs(std::string filePath) {
        FILE *fp = fopen(filePath.c_str(), "r");
        if (fp == NULL) {
            fprintf(stderr, "cannot open %s\n", filePath.c_str());
            exit(1);
        }
        double mae;
        std::vector<double> maes;
        while (fscanf(fp, "%lf", &mae) != EOF)
            maes.push_back(mae);
        fclose(fp);
        return maes;
    }

public:
    MAEClassifier(void):
        maxResidualError_(1.0),
        maeHistogramBinWidth_(0.01) {}

    inline void setClassifierDir(std::string classifiersDir) { classifiersDir_ = classifiersDir; }
    inline void setMaxResidualError(double maxResidualError) { maxResidualError_ = maxResidualError; }
    inline void setMAEHistogramBinWidth(double maeHistogramBinWidth) { maeHistogramBinWidth_ = maeHistogramBinWidth; }

    inline double getFailureThreshold(void) { return failureThreshold_; }

    double getMAE(std::vector<double> residualErrors) {
        double sum = 0.0;
        int num = 0;
        for (size_t i = 0; i < residualErrors.size(); i++) {
            if (0.0 <= residualErrors[i] && residualErrors[i] <= maxResidualError_) {
                sum += residualErrors[i];
                num++;
            }
        }
        if (num == 0)
            return 0.0;
        else
            return sum / (double)num;
    }

    void learnThreshold(std::vector<std::vector<double>> trainSuccessResidualErrors, std::vector<std::vector<double>> trainFailureResidualErrors) {
        std::vector<double> successMAEs((int)trainSuccessResidualErrors.size());
        std::vector<double> failureMAEs((int)trainFailureResidualErrors.size());
        for (int i = 0; i < (int)trainSuccessResidualErrors.size(); ++i)
            successMAEs[i] = getMAE(trainSuccessResidualErrors[i]);
        for (int i = 0; i < (int)trainFailureResidualErrors.size(); ++i)
            failureMAEs[i] = getMAE(trainFailureResidualErrors[i]);

        successMAEMean_ = getMean(successMAEs);
        successMAEStd_ = getStd(successMAEs);
        failureMAEMean_ = getMean(failureMAEs);
        failureMAEStd_ = getStd(failureMAEs);

        bool isFirst = true;
        double maxAcc;
        int totalNum = (int)trainSuccessResidualErrors.size() + (int)trainFailureResidualErrors.size();
        for (double th = successMAEMean_; th <= failureMAEMean_; th += 0.005) {
            int correctNum = 0;
            for (int i = 0; i < (int)trainSuccessResidualErrors.size(); ++i) {
                if (successMAEs[i] <= th)
                    correctNum++;
            }
            for (int i = 0; i < (int)trainFailureResidualErrors.size(); ++i) {
                if (failureMAEs[i] > th)
                    correctNum++;
            }
            double acc = (double)correctNum / (double)totalNum;
            printf("maeTH = %lf [m], accuracy = %lf [%%]\n", th, acc * 100.0);
            if (isFirst) {
                failureThreshold_ = th;
                maxAcc = acc;
                isFirst = false;
            } else if (maxAcc < acc) {
                failureThreshold_ = th;
                maxAcc = acc;
            }
        }
    }

    void writeClassifierParams(std::vector<std::vector<double>> testSuccessResidualErrors, std::vector<std::vector<double>> testFailureResidualErrors) {
        std::string mkdirCmd = "mkdir -p " + classifiersDir_; 
        int retVal = system(mkdirCmd.c_str());

        std::string positiveMAEsFileName = classifiersDir_ + "positive_maes.txt";
        std::string negativeMAEsFileName = classifiersDir_ + "negative_maes.txt";
        std::string truePositiveMAEsFileName = classifiersDir_ + "true_positive_maes.txt";
        std::string trueNegativeMAEsFileName = classifiersDir_ + "true_negative_maes.txt";
        std::string falsePositiveMAEsFileName = classifiersDir_ + "false_positive_maes.txt";
        std::string falseNegativeMAEsFileName = classifiersDir_ + "false_negative_maes.txt";
        FILE *fpPositive = fopen(positiveMAEsFileName.c_str(), "w");
        FILE *fpNegative = fopen(negativeMAEsFileName.c_str(), "w");
        FILE *fpTruePositive = fopen(truePositiveMAEsFileName.c_str(), "w");
        FILE *fpTrueNegative = fopen(trueNegativeMAEsFileName.c_str(), "w");
        FILE *fpFalsePositive = fopen(falsePositiveMAEsFileName.c_str(), "w");
        FILE *fpFalseNegative = fopen(falseNegativeMAEsFileName.c_str(), "w");

        truePositiveNum_ = falseNegativeNum_ = falsePositiveNum_ = trueNegativeNum_ = 0;
        for (size_t i = 0; i < testSuccessResidualErrors.size(); i++) {
            double mae = getMAE(testSuccessResidualErrors[i]);
            fprintf(fpPositive, "%lf\n", mae);
            if (mae <= failureThreshold_) {
                truePositiveNum_++;
                fprintf(fpTruePositive, "%lf\n", mae);
            } else {
                falseNegativeNum_++;
                fprintf(fpFalseNegative, "%lf\n", mae);
            }
        }
        for (size_t i = 0; i < testFailureResidualErrors.size(); i++) {
            double mae = getMAE(testFailureResidualErrors[i]);
            fprintf(fpNegative, "%lf\n", mae);
            if (mae <= failureThreshold_) {
                falsePositiveNum_++;
                fprintf(fpFalsePositive, "%lf\n", mae);
            } else {
                trueNegativeNum_++;
                fprintf(fpTrueNegative, "%lf\n", mae);
            }
        }
        fclose(fpPositive);
        fclose(fpNegative);
        fclose(fpTruePositive);
        fclose(fpTrueNegative);
        fclose(fpFalsePositive);
        fclose(fpFalseNegative);

        printf("truePositiveNum = %d\n", truePositiveNum_);
        printf("falseNegativeNum = %d\n", falseNegativeNum_);
        printf("trueNegativeNum = %d\n", trueNegativeNum_);
        printf("falsePositiveNum = %d\n", falsePositiveNum_);

        std::string yamlFile = classifiersDir_ + "classifier.yaml";
        FILE *fpYaml = fopen(yamlFile.c_str(), "w");
        fprintf(fpYaml, "maxResidualError: %lf\n", maxResidualError_);
        fprintf(fpYaml, "maeHistogramBinWidth: %lf\n", maeHistogramBinWidth_);
        fprintf(fpYaml, "failureThreshold: %lf\n", failureThreshold_);
        fprintf(fpYaml, "successMAEMean: %lf\n", successMAEMean_);
        fprintf(fpYaml, "successMAEStd: %lf\n", successMAEStd_);
        fprintf(fpYaml, "failureMAEMean: %lf\n", failureMAEMean_);
        fprintf(fpYaml, "failureMAEStd: %lf\n", failureMAEStd_);
        fprintf(fpYaml, "truePositiveNum: %d\n", truePositiveNum_);
        fprintf(fpYaml, "falseNegativeNum: %d\n", falseNegativeNum_);
        fprintf(fpYaml, "trueNegativeNum: %d\n", trueNegativeNum_);
        fprintf(fpYaml, "falsePositiveNum: %d\n", falsePositiveNum_);
        fprintf(fpYaml, "positiveMAEsFileName: positive_maes.txt\n");
        fprintf(fpYaml, "negativeMAEsFileName: negative_maes.txt\n");
        fprintf(fpYaml, "truePositiveMAEsFileName: true_positive_maes.txt\n");
        fprintf(fpYaml, "trueNegativeMAEsFileName: true_negative_maes.txt\n");
        fprintf(fpYaml, "falsePositiveMAEsFileName: false_positive_maes.txt\n");
        fprintf(fpYaml, "falseNegativeMAEsFileName: false_negative_maes.txt\n");
        fclose(fpYaml);
        printf("yaml file for the MAE classifier was saved at %s\n", yamlFile.c_str());
    }

    void readClassifierParams(void) {
        std::string yamlFile = classifiersDir_ + "classifier.yaml";
        YAML::Node lconf = YAML::LoadFile(yamlFile);

        maxResidualError_ = lconf["maxResidualError"].as<double>();
        failureThreshold_ = lconf["failureThreshold"].as<double>();
        maeHistogramBinWidth_ = lconf["maeHistogramBinWidth"].as<double>();

        std::string positiveMAEsFilePath = classifiersDir_ + lconf["positiveMAEsFileName"].as<std::string>();
        std::string negativeMAEsFilePath = classifiersDir_ + lconf["negativeMAEsFileName"].as<std::string>();
        std::string truePositiveMAEsFilePath = classifiersDir_ + lconf["truePositiveMAEsFileName"].as<std::string>();
        std::string trueNegativeMAEsFilePath = classifiersDir_ + lconf["trueNegativeMAEsFileName"].as<std::string>();
        std::string falsePositiveMAEsFilePath = classifiersDir_ + lconf["falsePositiveMAEsFileName"].as<std::string>();
        std::string falseNegativeMAEsFilePath = classifiersDir_ + lconf["falseNegativeMAEsFileName"].as<std::string>();
        positiveMAEHistogram_ = Histogram(readMAEs(positiveMAEsFilePath), maeHistogramBinWidth_);
        negativeMAEHistogram_ = Histogram(readMAEs(negativeMAEsFilePath), maeHistogramBinWidth_);
        truePositiveMAEHistogram_ = Histogram(readMAEs(truePositiveMAEsFilePath), maeHistogramBinWidth_);
        trueNegativeMAEHistogram_ = Histogram(readMAEs(trueNegativeMAEsFilePath), maeHistogramBinWidth_);
        falsePositiveMAEHistogram_ = Histogram(readMAEs(falsePositiveMAEsFilePath), maeHistogramBinWidth_);
        falseNegativeMAEHistogram_ = Histogram(readMAEs(falseNegativeMAEsFilePath), maeHistogramBinWidth_);

        positiveMAEHistogram_.smoothHistogram();
        negativeMAEHistogram_.smoothHistogram();
        truePositiveMAEHistogram_.smoothHistogram();
        trueNegativeMAEHistogram_.smoothHistogram();
        falsePositiveMAEHistogram_.smoothHistogram();
        falseNegativeMAEHistogram_.smoothHistogram();

        int truePositiveNum = lconf["truePositiveNum"].as<int>();
        int falseNegativeNum = lconf["falseNegativeNum"].as<int>();
        int trueNegativeNum = lconf["trueNegativeNum"].as<int>();
        int falsePositiveNum = lconf["falsePositiveNum"].as<int>();
        dTruePositive_ = (double)truePositiveNum / (double)(truePositiveNum + falseNegativeNum);
        dFalsePositive_ = (double)falsePositiveNum / (double)(truePositiveNum + falseNegativeNum);
        dTrueNegative_ = (double)trueNegativeNum / (double)(trueNegativeNum + falsePositiveNum);
        dFalseNegative_ = (double)falseNegativeNum / (double)(trueNegativeNum + falsePositiveNum);
    }

    double calculateDecisionModel(double mae, double *reliability) {
        double pSuccess, pFailure;
        pSuccess = dTruePositive_ * positiveMAEHistogram_.getProbability(mae) + dFalsePositive_ * positiveMAEHistogram_.getProbability(mae);
        pFailure = dTrueNegative_ * negativeMAEHistogram_.getProbability(mae) + dFalsePositive_ * negativeMAEHistogram_.getProbability(mae);
        if (pSuccess < 10.0e-9)
            pSuccess = 10.0e-9;
        if (pFailure < 10.0e-9)
            pFailure = 10.0e-9;
        double rel = pSuccess * *reliability;
        double relInv = pFailure * (1.0 - *reliability);
        double p = rel + relInv;
        if (p > 1.0)
            p = 1.0;
        *reliability = rel / (rel + relInv);
        if (*reliability > 0.9999)
            *reliability = 0.9999;
        if (*reliability < 0.0001)
            *reliability = 0.0001;
        return p;
    }

    void writeDecisionLikelihoods(void) {
        readClassifierParams();
        std::string fileName = classifiersDir_ + "mae_decision_likelihoods.txt";
        FILE *fp = fopen(fileName.c_str(), "w");
        for (double rel = 0.0; rel <= 1.0 + 0.05; rel += 0.05) {
            for (double mae = 0.0; mae <= 0.7 + maeHistogramBinWidth_; mae += maeHistogramBinWidth_) {
                double pSuccessPositive = dTruePositive_ * truePositiveMAEHistogram_.getProbability(mae);
                double pFailurePositive = dFalsePositive_ * falsePositiveMAEHistogram_.getProbability(mae);
                if (pSuccessPositive < 10.0e-6)
                    pSuccessPositive = 10.0e-6;
                if (pFailurePositive < 10.0e-6)
                    pFailurePositive = 10.0e-6;
                double relPositive = pSuccessPositive * rel;
                double relInvPositive = pFailurePositive * (1.0 - rel);
                double pPositive = relPositive + relInvPositive;

                double pSuccessNegative = dFalseNegative_ * falseNegativeMAEHistogram_.getProbability(mae);
                double pFailureNegative = dTrueNegative_ * trueNegativeMAEHistogram_.getProbability(mae);
                if (pSuccessNegative < 10.0e-6)
                    pSuccessNegative = 10.0e-6;
                if (pFailureNegative < 10.0e-6)
                    pFailureNegative = 10.0e-6;
                double relNegative = pSuccessNegative * rel;
                double relInvNegative = pFailureNegative * (1.0 - rel);
                double pNegative = relNegative + relInvNegative;

                fprintf(fp, "%lf %lf %lf %lf %lf\n", rel, mae, pPositive, pNegative, pPositive + pNegative);
            }
            fprintf(fp, "\n");
        }
        fclose(fp);
    }
}; // class MAEClassifier

} // namespace als_ros

#endif // __MAE_CLASSIFIER_H__

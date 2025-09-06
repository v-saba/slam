//
// Created by vas on 14/05/2025.
//

#ifndef SLAM_STEP_H
#define SLAM_STEP_H

#include <memory>
#include <opencv2/core.hpp>

namespace slam {

typedef std::vector<cv::Point3f> PointCloud;

class FrameCorrespondenceBuilder
{
public:
    FrameCorrespondenceBuilder();
    FrameCorrespondenceBuilder(float fx, float fy, float cx, float cy); // camera intrinsics
    FrameCorrespondenceBuilder(const std::string& calibration_file); // load from JSON file
    FrameCorrespondenceBuilder(const FrameCorrespondenceBuilder&) = delete;
    FrameCorrespondenceBuilder& operator=(const FrameCorrespondenceBuilder&) = delete;
    FrameCorrespondenceBuilder(FrameCorrespondenceBuilder&& other) noexcept;
    FrameCorrespondenceBuilder& operator=(FrameCorrespondenceBuilder&& other) noexcept;
    ~FrameCorrespondenceBuilder();

    // Load calibration from JSON file
    bool loadCalibration(const std::string& calibration_file);

    PointCloud buildCorrespondence(const cv::Mat& image) const; // gets next frame and builds correspondence

    struct Config {
        bool normalize_hist = true;
        enum class FeatureDetector { SIFT, ORB } feature_detector = FeatureDetector::SIFT;
        enum class Matcher { FLANN, BF } matcher = Matcher::FLANN;
        enum class EssentialMatrixEstimator { RANSAC, LMEDS } essential_matrix_estimator = EssentialMatrixEstimator::RANSAC;
        float match_threshold = 0.75f;
        bool enable_visualization = false;
    };
    Config _config;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
}

#endif //SLAM_STEP_H

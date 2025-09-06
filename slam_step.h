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
    FrameCorrespondenceBuilder(const FrameCorrespondenceBuilder&) = delete;
    FrameCorrespondenceBuilder& operator=(const FrameCorrespondenceBuilder&) = delete;
    FrameCorrespondenceBuilder(FrameCorrespondenceBuilder&& other) noexcept;
    FrameCorrespondenceBuilder& operator=(FrameCorrespondenceBuilder&& other) noexcept;
    ~FrameCorrespondenceBuilder();

    PointCloud buildCorrespondence(const cv::Mat& image) const; // gets next frame and builds correspondence

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
}

#endif //SLAM_STEP_H

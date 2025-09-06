//
// Created by vas on 14/05/2025.
//

#include "slam_step.h"

#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace slam {

struct FrameCorrespondenceBuilder::Impl {
    float fx = 900, fy = 900, cx = 640, cy = 360;  // Default camera intrinsics
    cv::Mat last_image;
    // feature detector
    cv::Ptr<cv::SIFT> sift;
    // matcher
    cv::Ptr<cv::FlannBasedMatcher> matcher;
    // correspondence
    PointCloud correspondence;
};

FrameCorrespondenceBuilder::FrameCorrespondenceBuilder(): _impl(std::make_unique<Impl>()) {}

FrameCorrespondenceBuilder::FrameCorrespondenceBuilder(float fx, float fy, float cx, float cy): _impl(std::make_unique<Impl>()) {
    _impl->fx = fx;
    _impl->fy = fy;
    _impl->cx = cx;
    _impl->cy = cy;
}

FrameCorrespondenceBuilder::FrameCorrespondenceBuilder(FrameCorrespondenceBuilder&& other) noexcept: _impl(std::move(other._impl)) {}

FrameCorrespondenceBuilder& FrameCorrespondenceBuilder::operator=(FrameCorrespondenceBuilder&& other) noexcept {
    if (this != &other) {
        _impl = std::move(other._impl);
    }
    return *this;
}

FrameCorrespondenceBuilder::~FrameCorrespondenceBuilder() = default;

PointCloud FrameCorrespondenceBuilder::buildCorrespondence(const cv::Mat &image) const
{
    if (_impl->last_image.empty()) {
        _impl->last_image = image;
        return {};
    }

    auto& last_image = _impl->last_image;
    assert(image.dims == last_image.dims);

    // Preprocessing - histogram equalization for better feature detection
    cv::Mat img1, img2;
    cv::equalizeHist(last_image, img1);
    cv::equalizeHist(image, img2);

    // Camera intrinsics from configuration
    cv::Mat K = (cv::Mat_<double>(3,3) << _impl->fx, 0, _impl->cx,
                 0, _impl->fy, _impl->cy,
                 0, 0, 1);

    // Detect SIFT keypoints & descriptors
    auto sift = cv::SIFT::create();
    std::vector<cv::KeyPoint> kp1, kp2;
    cv::Mat desc1, desc2;
    sift->detectAndCompute(img1, cv::noArray(), kp1, desc1);
    sift->detectAndCompute(img2, cv::noArray(), kp2, desc2);

    // Match descriptors using FLANN
    cv::FlannBasedMatcher matcher;
    std::vector<std::vector<cv::DMatch>> matches;
    // Convert descriptors to CV_32F for FLANN
    if(desc1.type() != CV_32F) {
        desc1.convertTo(desc1, CV_32F);
        desc2.convertTo(desc2, CV_32F);
    }
    matcher.knnMatch(desc1, desc2, matches, 2);
    std::vector<cv::DMatch> good_matches;
    for (const auto& knn : matches) {
        if (knn[0].distance < 0.75f * knn[1].distance)
            good_matches.push_back(knn[0]);
    }

    // Convert matches to points
    std::vector<cv::Point2f> pts1, pts2;
    for (const auto& m : good_matches) {
        pts1.push_back(kp1[m.queryIdx].pt);
        pts2.push_back(kp2[m.trainIdx].pt);
    }

    // Find essential matrix
    cv::Mat mask;
    cv::Mat E = cv::findEssentialMat(pts1, pts2, K, cv::RANSAC, 0.999, 1.0, mask);

    // Filter inlier points using RANSAC mask
    std::vector<cv::Point2f> inlier_pts1, inlier_pts2;
    for (int i = 0; i < mask.rows; ++i) {
        if (mask.at<uchar>(i)) {
            inlier_pts1.push_back(pts1[i]);
            inlier_pts2.push_back(pts2[i]);
        }
    }

    // Recover relative pose
    cv::Mat R, t;
    cv::recoverPose(E, inlier_pts1, inlier_pts2, K, R, t);

    // Triangulate
    cv::Mat P1 = K * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat Rt;
    cv::hconcat(R, t, Rt);
    cv::Mat P2 = K * Rt;

    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, inlier_pts1, inlier_pts2, pts4D);

    // Convert to 3D points
    std::vector<cv::Point3f> pointCloud;
    for (int i = 0; i < pts4D.cols; ++i) {
        cv::Mat col = pts4D.col(i);
        col /= col.at<float>(3);
        pointCloud.emplace_back(col.at<float>(0), col.at<float>(1), col.at<float>(2));
    }

    // Update last image for next frame
    _impl->last_image = image.clone();

    return pointCloud;
}

}

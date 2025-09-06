#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#include "slam_step.h"

void exportPointCloud(const std::vector<cv::Point3f>& pointCloud, const char* filename)
{
    // Save as PLY
    std::ofstream out(filename);
    out << "ply\nformat ascii 1.0\nelement vertex " << pointCloud.size()
        << "\nproperty float x\nproperty float y\nproperty float z"
        << "end_header\n";
    for (size_t i = 0; i < pointCloud.size(); ++i) {
        auto& p = pointCloud[i];
        out << p.x << " " << p.y << " " << p.z << "\n";
    }
    out.close();
}

int main(int argc, char* argv[])
{
    if (argc < 4 || argc > 5) {
        std::cerr << "Usage: " << argv[0] << " <image1_path> <image2_path> <output_pointcloud_path> [calibration_file.json]" << std::endl;
        std::cerr << "Examples:" << std::endl;
        std::cerr << "  " << argv[0] << " data/IMG_1956.jpg data/IMG_1957.jpg cloud.ply" << std::endl;
        std::cerr << "  " << argv[0] << " data/IMG_1956.jpg data/IMG_1957.jpg cloud.ply camera_calibration.json" << std::endl;
        return 1;
    }

    const char* img1_path = argv[1];
    const char* img2_path = argv[2];
    const char* output_path = argv[3];
    const char* calibration_path = (argc == 5) ? argv[4] : nullptr;

    std::cout << "Processing images: " << img1_path << " and " << img2_path << std::endl;
    std::cout << "Output will be saved to: " << output_path << std::endl;
    if (calibration_path) {
        std::cout << "Using calibration file: " << calibration_path << std::endl;
    }

    // Load images
    cv::Mat img1 = cv::imread(img1_path, cv::IMREAD_GRAYSCALE);
    cv::Mat img2 = cv::imread(img2_path, cv::IMREAD_GRAYSCALE);

    if (img1.empty() || img2.empty()) {
        std::cerr << "Error: Could not load images!" << std::endl;
        return 1;
    }

    // Create SLAM builder with calibration file or default values
    slam::FrameCorrespondenceBuilder slamBuilder = calibration_path ?
        slam::FrameCorrespondenceBuilder(calibration_path) :
        slam::FrameCorrespondenceBuilder(8064.0f, 8064.0f, 2016.0f, 1134.0f);  // iPhone estimates

    // Process first frame (initializes)
    slamBuilder.buildCorrespondence(img1);

    // Process second frame and get point cloud
    auto pointCloud = slamBuilder.buildCorrespondence(img2);

    exportPointCloud(pointCloud, output_path);

    std::cout << "Point cloud reconstruction complete using "
              << (calibration_path ? "calibrated" : "estimated") << " camera parameters!" << std::endl;
    return 0;
}

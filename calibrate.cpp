#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <chrono>

using json = nlohmann::json;
namespace fs = std::filesystem;

struct CalibrationConfig {
    cv::Size board_size{9, 6};  // Internal corners (width, height)
    float square_size = 25.0f;  // Size of squares in mm
    int min_images = 10;        // Minimum number of valid images for calibration
    bool show_detection = true; // Show detection results
    std::string output_file = "camera_calibration.json";
};

class CameraCalibrator {
private:
    CalibrationConfig config_;
    std::vector<std::vector<cv::Point2f>> image_points_;
    std::vector<std::vector<cv::Point3f>> object_points_;
    cv::Size image_size_;

    // Calibration results
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Mat> rvecs_, tvecs_;
    double reprojection_error_;

public:
    CameraCalibrator(const CalibrationConfig& config) : config_(config) {}

    // Generate 3D object points for checkerboard
    std::vector<cv::Point3f> generateObjectPoints() {
        std::vector<cv::Point3f> corners;
        for (int i = 0; i < config_.board_size.height; ++i) {
            for (int j = 0; j < config_.board_size.width; ++j) {
                corners.emplace_back(j * config_.square_size, i * config_.square_size, 0);
            }
        }
        return corners;
    }

    // Process a single image for calibration
    bool processImage(const cv::Mat& image, const std::string& image_name = "") {
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }

        // Store image size (needed for calibration)
        if (image_size_.width == 0) {
            image_size_ = gray.size();
        }

        // Find checkerboard corners
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, config_.board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (found) {
            // Refine corner positions
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            // Store points
            image_points_.push_back(corners);
            object_points_.push_back(generateObjectPoints());

            std::cout << "✓ Found checkerboard in " << (image_name.empty() ? "image" : image_name)
                      << " (" << image_points_.size() << "/" << config_.min_images << ")" << std::endl;

            // Visualize detection
            if (config_.show_detection) {
                cv::Mat display = image.clone();
                cv::drawChessboardCorners(display, config_.board_size, corners, found);
                cv::imshow("Checkerboard Detection", display);
                cv::waitKey(100);
            }
        } else {
            std::cout << "✗ No checkerboard found in " << (image_name.empty() ? "image" : image_name) << std::endl;
        }

        return found;
    }

    // Calibrate camera using collected points
    bool calibrate() {
        if (image_points_.size() < config_.min_images) {
            std::cerr << "Error: Need at least " << config_.min_images
                      << " valid images for calibration (got " << image_points_.size() << ")" << std::endl;
            return false;
        }

        std::cout << "\nPerforming camera calibration with " << image_points_.size() << " images..." << std::endl;

        // Initialize camera matrix with reasonable values
        camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix_.at<double>(0, 0) = image_size_.width;  // fx
        camera_matrix_.at<double>(1, 1) = image_size_.width;  // fy (assume square pixels)
        camera_matrix_.at<double>(0, 2) = image_size_.width / 2.0;   // cx
        camera_matrix_.at<double>(1, 2) = image_size_.height / 2.0;  // cy

        // Initialize distortion coefficients
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // Perform calibration
        reprojection_error_ = cv::calibrateCamera(object_points_, image_points_, image_size_,
            camera_matrix_, dist_coeffs_, rvecs_, tvecs_,
            cv::CALIB_FIX_PRINCIPAL_POINT | cv::CALIB_ZERO_TANGENT_DIST);

        std::cout << "Calibration complete!" << std::endl;
        std::cout << "Reprojection error: " << reprojection_error_ << " pixels" << std::endl;

        return true;
    }

    // Save calibration results to JSON file
    bool saveCalibration() {
        if (camera_matrix_.empty() || dist_coeffs_.empty()) {
            std::cerr << "Error: No calibration data to save" << std::endl;
            return false;
        }

        json calibration_data;

        // Camera matrix
        calibration_data["camera_matrix"] = {
            {"fx", camera_matrix_.at<double>(0, 0)},
            {"fy", camera_matrix_.at<double>(1, 1)},
            {"cx", camera_matrix_.at<double>(0, 2)},
            {"cy", camera_matrix_.at<double>(1, 2)}
        };

        // Distortion coefficients
        std::vector<double> dist_vec;
        for (int i = 0; i < dist_coeffs_.rows; ++i) {
            dist_vec.push_back(dist_coeffs_.at<double>(i, 0));
        }
        calibration_data["distortion_coefficients"] = dist_vec;

        // Image size
        calibration_data["image_size"] = {
            {"width", image_size_.width},
            {"height", image_size_.height}
        };

        // Calibration metadata
        calibration_data["metadata"] = {
            {"reprojection_error", reprojection_error_},
            {"num_images", image_points_.size()},
            {"board_size", {config_.board_size.width, config_.board_size.height}},
            {"square_size_mm", config_.square_size},
            {"calibration_date", std::chrono::system_clock::to_time_t(std::chrono::system_clock::now())}
        };

        // Save to file
        std::ofstream file(config_.output_file);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open file for writing: " << config_.output_file << std::endl;
            return false;
        }

        file << calibration_data.dump(4) << std::endl;
        file.close();

        std::cout << "Calibration saved to: " << config_.output_file << std::endl;
        return true;
    }

    // Print calibration results
    void printResults() {
        if (camera_matrix_.empty()) return;

        std::cout << "\n=== Camera Calibration Results ===" << std::endl;
        std::cout << "Image size: " << image_size_.width << "x" << image_size_.height << std::endl;
        std::cout << "Focal length (fx, fy): " << camera_matrix_.at<double>(0, 0)
                  << ", " << camera_matrix_.at<double>(1, 1) << std::endl;
        std::cout << "Principal point (cx, cy): " << camera_matrix_.at<double>(0, 2)
                  << ", " << camera_matrix_.at<double>(1, 2) << std::endl;
        std::cout << "Distortion coefficients: ";
        for (int i = 0; i < dist_coeffs_.rows; ++i) {
            std::cout << dist_coeffs_.at<double>(i, 0);
            if (i < dist_coeffs_.rows - 1) std::cout << ", ";
        }
        std::cout << std::endl;
        std::cout << "Reprojection error: " << reprojection_error_ << " pixels" << std::endl;
        std::cout << "=================================" << std::endl;
    }
};

void printUsage(const char* program_name) {
    std::cout << "Camera Calibration Tool" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << program_name << " <mode> [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Modes:" << std::endl;
    std::cout << "  images <directory>     - Calibrate using images from directory" << std::endl;
    std::cout << "  live [camera_id]       - Calibrate using live camera (default id: 0)" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --board-size WxH       - Checkerboard size (default: 9x6)" << std::endl;
    std::cout << "  --square-size SIZE     - Square size in mm (default: 25.0)" << std::endl;
    std::cout << "  --min-images N         - Minimum images needed (default: 10)" << std::endl;
    std::cout << "  --output FILE          - Output JSON file (default: camera_calibration.json)" << std::endl;
    std::cout << "  --no-display           - Don't show detection visualization" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << program_name << " images data/calibration/" << std::endl;
    std::cout << "  " << program_name << " live 0 --board-size 7x5 --square-size 30" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    CalibrationConfig config;
    std::string mode = argv[1];

    // Parse command line arguments
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--board-size" && i + 1 < argc) {
            std::string size_str = argv[++i];
            int pos = size_str.find('x');
            if (pos != std::string::npos) {
                config.board_size.width = std::stoi(size_str.substr(0, pos));
                config.board_size.height = std::stoi(size_str.substr(pos + 1));
            }
        } else if (arg == "--square-size" && i + 1 < argc) {
            config.square_size = std::stof(argv[++i]);
        } else if (arg == "--min-images" && i + 1 < argc) {
            config.min_images = std::stoi(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            config.output_file = argv[++i];
        } else if (arg == "--no-display") {
            config.show_detection = false;
        }
    }

    CameraCalibrator calibrator(config);

    if (mode == "images") {
        if (argc < 3) {
            std::cerr << "Error: Directory path required for images mode" << std::endl;
            return 1;
        }

        std::string directory = argv[2];
        if (!fs::exists(directory)) {
            std::cerr << "Error: Directory does not exist: " << directory << std::endl;
            return 1;
        }

        std::cout << "Loading calibration images from: " << directory << std::endl;
        std::cout << "Checkerboard size: " << config.board_size.width << "x" << config.board_size.height << std::endl;
        std::cout << "Square size: " << config.square_size << "mm" << std::endl;

        // Process all images in directory
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.is_regular_file()) {
                std::string path = entry.path().string();
                std::string ext = entry.path().extension().string();

                // Check if it's an image file
                if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".tiff") {
                    cv::Mat image = cv::imread(path);
                    if (!image.empty()) {
                        calibrator.processImage(image, entry.path().filename().string());
                    }
                }
            }
        }

    } else if (mode == "live") {
        int camera_id = 0;
        if (argc >= 3) {
            camera_id = std::stoi(argv[2]);
        }

        cv::VideoCapture cap(camera_id);
        if (!cap.isOpened()) {
            std::cerr << "Error: Could not open camera " << camera_id << std::endl;
            return 1;
        }

        std::cout << "Live camera calibration (Camera ID: " << camera_id << ")" << std::endl;
        std::cout << "Press SPACE to capture image, 'q' to start calibration, ESC to quit" << std::endl;

        cv::Mat frame;
        while (true) {
            cap >> frame;
            if (frame.empty()) break;

            cv::imshow("Camera Calibration - Press SPACE to capture", frame);

            int key = cv::waitKey(30) & 0xFF;
            if (key == 27) {  // ESC
                std::cout << "Calibration cancelled" << std::endl;
                return 0;
            } else if (key == ' ') {  // SPACE
                calibrator.processImage(frame);
            } else if (key == 'q') {  // Start calibration
                break;
            }
        }

    } else {
        std::cerr << "Error: Unknown mode '" << mode << "'" << std::endl;
        printUsage(argv[0]);
        return 1;
    }

    // Perform calibration
    if (calibrator.calibrate()) {
        calibrator.printResults();
        calibrator.saveCalibration();
        std::cout << "\nCalibration completed successfully!" << std::endl;
    } else {
        std::cerr << "Calibration failed!" << std::endl;
        return 1;
    }

    cv::destroyAllWindows();
    return 0;
}

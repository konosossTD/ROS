#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

int main() {
    // 棋盘格大小 (内角点数)
    cv::Size boardSize(9, 6); // 9x6 棋盘格
    float squareSize = 25.0; // 每个格子的大小（毫米），根据打印尺寸设置

    // 读取标定图片路径
    std::vector<std::string> imagePaths = {
        "/home/may/Desktop/ROS/learn/src/orbslam/cali/1.jpg",
        "/home/may/Desktop/ROS/learn/src/orbslam/cali/2.jpg",
        "/home/may/Desktop/ROS/learn/src/orbslam/cali/3.jpg",
        "/home/may/Desktop/ROS/learn/src/orbslam/cali/4.jpg"};

    // 存储角点
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // 3D 世界坐标系的角点
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; ++i)
        for (int j = 0; j < boardSize.width; ++j)
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

    // 提取角点
    for (const auto &path : imagePaths) {
        cv::Mat image = cv::imread(path);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << path << std::endl;
            continue;
        }

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            image, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            cv::Mat gray;
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS +
                                                  cv::TermCriteria::COUNT,
                                              30, 0.1));
            imagePoints.push_back(corners);
            objectPoints.push_back(obj);
        }
    }

    // 标定相机
    cv::Mat cameraMatrix, distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;
    cv::calibrateCamera(objectPoints, imagePoints, boardSize, cameraMatrix,
                        distCoeffs, rvecs, tvecs);

    // 输出结果
    std::cout << "Camera Matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << distCoeffs << std::endl;

    return 0;
}

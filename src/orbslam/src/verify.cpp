#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    // 加载相机内参和畸变系数
    cv::Mat cameraMatrix =
        (cv::Mat_<double>(3, 3) << 598.5198187444446, 0, 374.0872506791089, 0,
         603.5628833804434, 335.8581290685513, 0, 0, 1);
    cv::Mat distCoeffs =
        (cv::Mat_<double>(1, 5) << -0.06970632428900425, 0.1853704048038906,
         0.0003575542815263582, 0.0004125574131786216, -0.1063739406852059);

    // 读取测试图像
    cv::Mat image =
        cv::imread("/home/may/Desktop/ROS/learn/src/orbslam/cali/3.jpg");
    if (image.empty()) {
        std::cerr << "Failed to load test image." << std::endl;
        return -1;
    }

    // 去畸变
    cv::Mat undistortedImage;
    cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

    // 显示结果
    cv::imshow("Original Image", image);
    cv::imshow("Undistorted Image", undistortedImage);
    cv::waitKey(0);

    return 0;
}

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char **argv) {
    // 初始化 ROS
    ros::init(argc, argv, "usb_cam_publisher");
    ros::NodeHandle nh;

    // 使用 image_transport 发布图像消息
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 1);

    // 打开 USB 摄像头
    cv::VideoCapture cap(0); // 打开默认的摄像头 (通常是 USB 摄像头)
    if (!cap.isOpened()) {
        ROS_ERROR("Failed to open camera.");
        return -1;
    } else {
        ROS_INFO("640 480 30FPS INIT DONE.");

        // 设置分辨率和帧率
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);  // 设置宽度
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // 设置高度
        cap.set(cv::CAP_PROP_FPS, 30);           // 设置帧率
    }

    cv::Mat frame;
    ros::Rate loop_rate(30); // 设置循环频率 30 Hz

    while (ros::ok()) {
        // 捕获一帧图像
        cap >> frame;
        if (frame.empty()) {
            ROS_ERROR("Captured empty frame.");
            break;
        }

        // 将 OpenCV 图像转换为 ROS 图像消息
        sensor_msgs::ImagePtr msg =
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        // 发布图像
        pub.publish(msg);

        // 在屏幕上显示图像（可选）
        // cv::imshow("Camera", frame);
        if (cv::waitKey(10) == 27) {
            break; // 按下 'ESC' 键退出
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cap.release();
    cv::destroyAllWindows();

    return 0;
}

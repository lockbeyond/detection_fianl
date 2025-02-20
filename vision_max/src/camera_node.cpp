//葛文扬 2024215592
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

using namespace cv;
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

class CameraNode : public rclcpp::Node {
public:
 std::string camera_info_url = "package://vision_max/config/camera_info.yaml";
    CameraNode() : Node("camera_node"), 
                   camera_info_manager_(this, "camera", camera_info_url) {
        // 初始化相机参数
        camera_info_ = camera_info_manager_.getCameraInfo();
        camera_pub_ = image_transport::create_camera_publisher(this, "image_raw");
        
        
        cap_.open("/home/locjbeyond/vision/output.avi");
        timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {
            cv::Mat frame;
            if (!cap_.read(frame)) {
                RCLCPP_WARN(this->get_logger(), "视频流结束");
                return;
            }
            
            // 发布图像和相机信息
            auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
            img_msg->header.stamp = now();
            img_msg->header.frame_id = "camera";
            camera_info_.header = img_msg->header;
            camera_pub_.publish(*img_msg, camera_info_);
        });
    }

private:
    image_transport::CameraPublisher camera_pub_;
    camera_info_manager::CameraInfoManager camera_info_manager_;
    sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}
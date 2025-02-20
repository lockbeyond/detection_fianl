#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <Eigen/Dense>  
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class PredictorNode : public rclcpp::Node {
public:
    PredictorNode() : Node("predictor_node"), last_time_(this->now()) {
        target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
            "target_3d", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                predictAimPoint(msg);
            });
        aim_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("aim_point", 10);
    }

private:
     void predictAimPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        auto current_time = msg->header.stamp;

        rclcpp::Time current_time_rclcpp(current_time); // 转换为 rclcpp::Time
        rclcpp::Duration dt = current_time_rclcpp - last_time_; // 计算时间差
        double dt_seconds = dt.seconds(); // 获取以秒为单位的时间差
        
        // 计算速度
        Eigen::Vector3d current_pos(msg->point.x, msg->point.y, msg->point.z);
        if (dt.seconds() > 1e-3) { // 避免除零
            velocity_ = (current_pos - last_pos_) / dt.seconds();
        }
        
        // 预测逻辑
        double flight_time = current_pos.norm() / 15.0;
        Eigen::Vector3d aim_pos = current_pos + velocity_ * flight_time;
        
        // 发布预瞄点
        auto aim_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
        aim_msg->header = msg->header;
        aim_msg->point.x = aim_pos.x();
        aim_msg->point.y = aim_pos.y();
        aim_msg->point.z = aim_pos.z();
        aim_pub_->publish(*aim_msg);
        
        // 更新状态
        last_pos_ = current_pos;
        last_time_ = current_time;
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr aim_pub_;
    Eigen::Vector3d last_pos_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
    rclcpp::Time last_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PredictorNode>());
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/calib3d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "stdio.h"
#include<iostream> 
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/dnn.hpp>
#include<opencv2/imgproc.hpp>
#include <opencv2/ml/ml.hpp>
using namespace message_filters;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> ApproximateTimePolicy;
using namespace cv::ml;
using namespace std;
using namespace cv;

class ArmorDetector {
public:
    ~ArmorDetector();
float width, length, angle, area;
      cv::Point2f center;
    ArmorDetector() ;
    //让得到的灯条套上一个旋转矩形，以方便之后对角度这个特殊因素作为匹配标准
    ArmorDetector(const cv::RotatedRect& light)
    {
        width = light.size.width;
        length = light.size.height;
        center = light.center;
        angle = light.angle;
        area = light.size.area();
    }
    void processFrame(const cv::Mat& frame);
    void displayResults(const cv::Mat& frame);
    void detectLampBars(const cv::Mat& frame);
};


ArmorDetector::ArmorDetector() {
    // 初始化代码
}

ArmorDetector::~ArmorDetector() {
    // 清理代码
}

void ArmorDetector::processFrame(const cv::Mat& frame) {
    detectLampBars(frame);
}

void ArmorDetector::displayResults(const cv::Mat& frame) {
    // 显示结果
}

void ArmorDetector::detectLampBars(const cv::Mat& frame) {
    Mat channels[3], binary, Gaussian, dilatee,gray;
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    Rect boundRect;
    RotatedRect box;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<Point2f> boxPts(4);
      cvtColor(frame, gray, COLOR_BGR2GRAY);
        threshold(gray, binary, 220, 255, 0);//二值化
        GaussianBlur(binary, Gaussian, Size(5, 5), 0);//滤波
        dilate(Gaussian, dilatee, element);
        // dilate(Gaussian, dilate, element, Point(-1, -1));//膨胀，把滤波得到的细灯条变宽
        findContours(dilatee, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);//轮廓检测
        vector<ArmorDetector> lightInfos;//创建一个灯条类的动态数组
        for (int i = 0; i < contours.size(); i++) {
            // 求轮廓面积
            double area = contourArea(contours[i]);
            // 去除较小轮廓&fitEllipse的限制条件
            if (area < 5|| contours[i].size() <= 1)
                continue;//相当于就是把这段轮廓去除掉
            // 用椭圆拟合区域得到外接矩形（特殊的处理方式：因为灯条是椭圆型的，所以用椭圆去拟合轮廓，再直接获取旋转外接矩形即可）
            RotatedRect Light_Rec = fitEllipse(contours[i]);
            // 长宽比和轮廓面积比限制（由于要考虑灯条的远近都被识别到，所以只需要看比例即可）
            if (Light_Rec.size.width / Light_Rec.size.height > 4)
                continue;
            lightInfos.push_back(ArmorDetector(Light_Rec));
        }

        for (size_t i = 0; i < lightInfos.size(); i++) {
            for (size_t j = i + 1; (j < lightInfos.size()); j++) {
                ArmorDetector& leftLight = lightInfos[i];
                ArmorDetector& rightLight = lightInfos[j];
                float angleGap_ = abs(leftLight.angle - rightLight.angle);
                //由于灯条长度会因为远近而受到影响，所以按照比值去匹配灯条
                float LenGap_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                float dis = pow(pow((leftLight.center.x - rightLight.center.x), 2) + pow((leftLight.center.y - rightLight.center.y), 2), 0.5);
                //均长
                float meanLen = (leftLight.length + rightLight.length) / 2;
                float lengap_ratio = abs(leftLight.length - rightLight.length) / meanLen;
                float yGap = abs(leftLight.center.y - rightLight.center.y);
                float yGap_ratio = yGap / meanLen;
                float xGap = abs(leftLight.center.x - rightLight.center.x);
                float xGap_ratio = xGap / meanLen;
                float ratio = dis / meanLen;
                //匹配不通过的条件
                if (angleGap_ > 5 ||
                    LenGap_ratio > 1.0 ||
                    lengap_ratio > 0.8 ||
                    yGap_ratio > 1.5 ||
                    xGap_ratio > 2.2 ||
                    xGap_ratio < 0.8 ||
                    ratio > 3 ||
                    ratio < 0.8) {
                    continue;
                }
                //绘制矩形
                Point center = Point((leftLight.center.x + rightLight.center.x) / 2, (leftLight.center.y + rightLight.center.y) / 2);
                RotatedRect rect = RotatedRect(center, Size(dis, meanLen), (leftLight.angle + rightLight.angle) / 2);
                Point2f vertices[4];
                rect.points(vertices);
                for (int i = 0; i < 4; i++) {
                    line(frame, vertices[i], vertices[(i + 1) % 4], Scalar(0, 0, 255), 2.2);
                }
            }
        }

        
}
//上面的是中期考核的代码直接粘贴过来了.....
using namespace std;
using namespace cv;


#include <rclcpp/rclcpp.hpp>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <message_filters/subscriber.h>

using namespace message_filters;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> MySyncPolicy;

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode() : Node("detection_node") {
        // 初始化订阅者
        image_sub_.subscribe(this, "image_raw");
        info_sub_.subscribe(this, "camera_info");
        aim_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "aim_point",
            rclcpp::SensorDataQoS(),
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
                latest_aim_ = msg;
            }
        );
 
        // 初始化发布者（正确位置！）
        target_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("target_3d", 10);
        viz_pub_ = image_transport::create_publisher(this, "image_viz");

        // 同步图像和相机参数
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), image_sub_, info_sub_);
        sync_->registerCallback(&DetectionNode::detectAndVisualize, this);
    }

private:
    void detectAndVisualize(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
                           const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, "bgr8");
            cv::Mat frame = cv_ptr->image;
                ArmorDetector detector;
    detector.detectLampBars(frame);
            // --------------------- 1. 装甲板检测 ---------------------
            std::vector<cv::Point2f> corners = detectArmor(frame);

            // PnP 解算并发布 /target_3d 
            if (!corners.empty()) {
                // 定义装甲板3D坐标
                std::vector<cv::Point3f> object_pts = {
                    {-0.13, -0.057, 0}, {0.13, -0.057, 0},
                    {0.13, 0.057, 0}, {-0.13, 0.057, 0}
                };
                // 提取相机参数
                cv::Mat camera_matrix(3, 3, CV_64F, const_cast<double*>(info_msg->k.data()));
                cv::Mat dist_coeffs(5, 1, CV_64F, const_cast<double*>(info_msg->d.data()));
                // 解算位姿
                cv::Mat rvec, tvec;
                if (cv::solvePnP(object_pts, corners, camera_matrix, dist_coeffs, rvec, tvec)) {
                    auto target_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
                    target_msg->header = img_msg->header;
                    target_msg->point.x = tvec.at<double>(0);
                    target_msg->point.y = tvec.at<double>(1);
                    target_msg->point.z = tvec.at<double>(2);
                    target_pub_->publish(*target_msg);  // 发布坐标
                }
            }

            //绘制预瞄点 
            if (latest_aim_) {
                cv::Matx34d proj_matrix(info_msg->p.data());
                cv::Vec3d aim_3d(latest_aim_->point.x, latest_aim_->point.y, latest_aim_->point.z);
                cv::Matx41d aim_homogeneous(aim_3d(0), aim_3d(1), aim_3d(2), 1.0);
                cv::Matx31d projected = proj_matrix * aim_homogeneous;
                if (projected(2) != 0) {
                    cv::Point2f aim_2d(projected(0)/projected(2), projected(1)/projected(2));
                    cv::circle(frame, aim_2d, 10, cv::Scalar(0,0,255), -1);
                }
            }

            // 发布标注图像
            cv_bridge::CvImage out_msg(img_msg->header, "bgr8", frame);
            viz_pub_.publish(out_msg.toImageMsg());
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
std::vector<cv::Point2f> detectArmor(cv::Mat& frame) {
    std::vector<cv::Point2f> armor_corners;
    cv::Mat hsv, mask, gray, binary;
    
    // 
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    // 定义红色和蓝色的HSV范围（
    cv::Scalar lower_red1(0, 120, 70), upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 120, 70), upper_red2(180, 255, 255);
    cv::Scalar lower_blue(100, 120, 70), upper_blue(130, 255, 255);
    // 提取红色区域（合并两个区间）
    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv, lower_red1, upper_red1, mask_red1);
    cv::inRange(hsv, lower_red2, upper_red2, mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);
    // 提取蓝色区域
    cv::Mat mask_blue;
    cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
    
    cv::bitwise_or(mask_red, mask_blue, mask);

    // 去除噪声
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // 轮廓检测与筛选 
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> light_bars;
    for (const auto& contour : contours) {
        //过滤小面积区域
        if (cv::contourArea(contour) < 100) continue;

        // 拟合旋转矩形
        cv::RotatedRect rect = cv::minAreaRect(contour);
        float width = rect.size.width;
        float height = rect.size.height;

        // 根据长宽比筛选灯条
        float aspect_ratio = std::max(width, height) / std::min(width, height);
        if (aspect_ratio < 4.0 || aspect_ratio > 15.0) continue;
        // 保存候选灯条
        light_bars.push_back(rect);
    }

    // 3. 灯条配对为装甲板 
    for (size_t i = 0; i < light_bars.size(); ++i) {
        for (size_t j = i + 1; j < light_bars.size(); ++j) {
            const cv::RotatedRect& rect1 = light_bars[i];
            const cv::RotatedRect& rect2 = light_bars[j];

            // 计算两个灯条的角度差（需近似平行）
            float angle_diff = std::abs(rect1.angle - rect2.angle);
            if (angle_diff > 10.0 && angle_diff < 170.0) continue; // 排除非平行或对顶角

            // 计算中心点距离（符合装甲板宽度范围）
            cv::Point2f center1 = rect1.center;
            cv::Point2f center2 = rect2.center;
            float distance = cv::norm(center1 - center2);
            if (distance < 50.0 || distance > 300.0) continue;

            // 计算高度差（灯条高度应相近）
            float height_diff = std::abs(rect1.size.height - rect2.size.height);
            if (height_diff > 20.0) continue;

            // -------------------- 4. 生成装甲板角点 --------------------
            // 获取灯条矩形的四个顶点
            cv::Point2f vertices1[4], vertices2[4];
            rect1.points(vertices1);
            rect2.points(vertices2);

            // 确定左右灯条（根据中心点x坐标）
            cv::RotatedRect left_rect = (center1.x < center2.x) ? rect1 : rect2;
            cv::RotatedRect right_rect = (center1.x < center2.x) ? rect2 : rect1;

            // 提取内侧的两个顶点（靠近装甲板中心）
            std::vector<cv::Point2f> left_inner, right_inner;
            for (int k = 0; k < 4; ++k) {
                if (vertices1[k].x > center1.x) left_inner.push_back(vertices1[k]);
                if (vertices2[k].x < center2.x) right_inner.push_back(vertices2[k]);
            }

            // 计算装甲板的四个角点（左上、右上、右下、左下）
            if (!left_inner.empty() && !right_inner.empty()) {
                cv::Point2f tl = left_inner[1];  // 左上
                cv::Point2f bl = left_inner[0];  // 左下
                cv::Point2f tr = right_inner[1]; // 右上
                cv::Point2f br = right_inner[0]; // 右下

                armor_corners = {tl, tr, br, bl};
                return armor_corners; 
            }
        }
    }

    return armor_corners; // 未检测到返回空
}

    // 成员变量
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr aim_sub_;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    image_transport::Publisher viz_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
    geometry_msgs::msg::PointStamped::SharedPtr latest_aim_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionNode>());
    rclcpp::shutdown();
    return 0;
}
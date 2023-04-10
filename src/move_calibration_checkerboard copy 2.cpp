/*
    需求：订阅图像数据，每间隔1s发送服务请求，修改checker_name的位姿
    步骤：
        1. 包含头文件
        2. 创建客户端类
            2.1 利用构造函数创建客户端对象
            2.2 定义：等待连接服务端函数
            2.3 定义：发送服务请求函数
        3. 创建订阅者回调函数，将传感器发送的消息转化为opencv兼容的cvimage（或者创建订阅者类）
        4. 主函数
            4.1 创建客户端实例化对象
            4.2 创建订阅者，订阅图像消息
            4.3 客户端发送服务请求函数（因为相机以30hz向外发布数据，订阅者持续订阅到消息，所以，在这里采用spin_once()，每隔1s回调一次）
            4.4 释放资源
*/

#include <iostream>
#include <math.h>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include "cv_bridge/cv_bridge.h"

using gazebo_msgs::srv::SetEntityState;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Quaternion;
using sensor_msgs::msg::Image;

using namespace std;
using namespace std::chrono_literals;
using namespace std::placeholders;

#include <signal.h>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/msg/image.hpp>

class SLAMNode: public rclcpp::Node
{
public:
    SLAMNode(): rclcpp::Node("test"), sync_policy_(10), image_sync_(sync_policy_) {
        auto subscribe_image_topic = [this](ImageSubscriberFilter &sub, std::string topic, const std::string = "raw") {
            sub.subscribe(this, topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
        };
        subscribe_image_topic(sub_image_, "image");
        subscribe_image_topic(sub_depth_, "depth");
        image_sync_.connectInput(sub_image_, sub_depth_);
        image_sync_.registerCallback(
            std::bind(&SLAMNode::rgbd_callback, this, std::placeholders::_1, std::placeholders::_2));
    };
    ~SLAMNode(){};

    void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr msgRGB,
                       const sensor_msgs::msg::Image::ConstSharedPtr msgD) {
        std::cout << "Callback " << std::endl;
    }

private:
    typedef message_filters::Subscriber<sensor_msgs::msg::Image> ImageSubscriberFilter;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> sync_policy;
    sync_policy sync_policy_;
    message_filters::Synchronizer<sync_policy> image_sync_;
    ImageSubscriberFilter sub_image_, sub_depth_, sub_right_image_, sub_mono_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<SLAMNode> node = std::make_shared<SLAMNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return EXIT_SUCCESS;
}

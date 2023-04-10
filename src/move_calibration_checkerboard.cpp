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
#include "message_filters/time_synchronizer.h"

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
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

// 定义opencv数据格式
cv::Mat RGB_img, IR_img, Depth_img;
// 定义一些flag
bool RGB_update_flag = 0, IR_update_flag = 0, Depth_update_flag = 0;

// 订阅话题消息名称
const string topic_ir = "/camera/camera_r/custom_image";
const string topic_rgb = "/camera/camera_l/custom_image";
const string topic_depth = "/camera/camera_r/custom_image_depth";
const char *save_dir = "./src/camera_calibration/save_checkboard_img/%s/%d.png";

class Sub_Image: public rclcpp::Node
{
public:
    Sub_Image(): rclcpp::Node("Sub_Image_node_cpp") {   
        sub_rgb.subscribe(this, topic_rgb);
        sub_ir.subscribe(this, topic_ir);
        sub_depth.subscribe(this, topic_depth);

        image_sync_ = std::make_shared<message_filters::TimeSynchronizer<Image, Image, Image>>(sub_rgb, sub_ir, sub_depth, 200);
        image_sync_->registerCallback(std::bind(&Sub_Image::rgbd_callback, this, _1, _2, _3));
    };

    void rgbd_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg_rgb,
                       const sensor_msgs::msg::Image::ConstSharedPtr msg_ir,
                       const sensor_msgs::msg::Image::ConstSharedPtr msg_depth) {
        RGB_update_flag = 0;
        IR_update_flag = 0;
        Depth_update_flag = 0;
        try
        {
            RGB_img = cv_bridge::toCvShare(msg_rgb, "bgr8")->image.clone();
            RGB_update_flag = 1;

            /* 可选择是否显示图片 */
            // cv::imshow("rgb_img", RGB_img);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调rgb出错");
            // ROS_ERROR("rgb Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        try
        {
            IR_img = cv_bridge::toCvShare(msg_ir, "bgr8")->image.clone();
            IR_update_flag = 1;
            /* 可选择是否显示图片 */
            // cv::imshow("ir_img", IR_img);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调ir出错");
            // RCLCPP_ERROR("ir Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }

        try
        {
            cv::Mat Depth_img_32FC1 = cv_bridge::toCvShare(msg_depth, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); //得到的是32FC1的图片 对应type为5
            Depth_img_32FC1.convertTo(Depth_img, CV_16U, 65535.0 / 1.0, 0.0);
            Depth_update_flag = 1;
            /* 可选择是否显示图片 */
            // cv::imshow("Depth_img", Depth_img);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调depth出错");
            // ROS_ERROR("depth Could not convert from depth_registered_Callback.");
        }
    }

private:
    message_filters::Subscriber<Image> sub_rgb;
    message_filters::Subscriber<Image> sub_ir;
    message_filters::Subscriber<Image> sub_depth;
    // 这里TimeSynchronizer应该定义成成员变量，而不能定义成局部变量；
    std::shared_ptr<message_filters::TimeSynchronizer<Image, Image, Image>> image_sync_;
};

// 创建客户端
class SetCheckerState: public rclcpp::Node
{
public:
    // 2.1 利用构造函数创建客户端对象
    SetCheckerState():Node("setcheckerstate_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建客户端节点");
        client_ = this->create_client<SetEntityState>("set_entity_state");
    }
    // 2.2 定义：等待连接服务端函数
    bool connect_server(){
        while (!client_->wait_for_service(2s))
        {
            if(!rclcpp::ok()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中");
        }
        return true;
    }
    // 2.3 定义：发送服务请求函数
    rclcpp::Client<SetEntityState>::FutureAndRequestId send_request(string name, Pose pose, Twist twist, string reference_frame){
        auto request = std::make_shared<SetEntityState::Request>();
        
        request->state.name = name;
        request->state.pose = pose;
        request->state.twist = twist;
        request->state.reference_frame = reference_frame;

        return client_->async_send_request(request);
    }

private:
    rclcpp::Client<SetEntityState>::SharedPtr client_;
};

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc,argv);
    // 4.1 创建客户端实例化对象，订阅者实例化对象
    auto client = std::make_shared<SetCheckerState>();
    auto sub_image = std::make_shared<Sub_Image>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "尝试连接服务器");
    bool flag = client->connect_server();
    if (!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败");
        return 0;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接成功");
    }

    rclcpp::executors::SingleThreadedExecutor exectuor;
    exectuor.add_node(sub_image);

    // 定义一些变量用于随后产生标定板的位置
    Twist twist;

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    Pose pose;
    Quaternion quat;

    double x_bias = -0.04;
    double y_bias = -0.04;
    double z_bias = 0.2;

    double dx = 0.3;
    double dy = 0.2;
    double dz = 0.25;

    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
    quat.w = 1.0;
    pose.orientation = quat;
    
    double x, y, z, qx, qy, qz, qw;

    // 设置客户端更改标定版位置的的频率
    rclcpp::Rate rate(1.0);

    // 4.3 客户端发送服务请求函数（因为相机以30hz向外发布数据，订阅者持续订阅到消息，所以，在这里采用spin_once()，每隔1s回调一次）
    while (rclcpp::ok())
    {
        // 随机产生一组四元数
        qx = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qy = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qz = 0.25 * (((rand() % 100) / 100.0) - 0.5);
        qw = 0.5;

        // 随机产生一组位置
        x = x_bias + dx * ((rand() % 100) / 100.0 - 0.5);
        y = y_bias + dy * ((rand() % 100) / 100.0 - 0.5);
        z = z_bias + dz * ((rand() % 100) / 100.0 - 0.5);

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;

        // 计算四元数？
        double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        quat.x = qx / norm;
        quat.y = qy / norm;
        quat.z = qz / norm;
        quat.w = qw / norm;       

        cout << "qx, qy, qz, qw= " << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << endl;
        cout << "x,y,z = " << x << ", " << y << ", " << z << endl;

        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = quat;
        
        // 客户端向gazebo服务端发送消息
        auto future = client->send_request("checker_name", pose, twist, "world");
        if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS){ // 成功
            RCLCPP_INFO(client->get_logger(), "响应成功");
        }
        else{
            RCLCPP_INFO(client->get_logger(), "响应失败");
        }

        rate.sleep();
        exectuor.spin_some();

        if(IR_update_flag && IR_update_flag && Depth_update_flag){
            // 说明成功订阅到数据，下面使用opencv判断图像信息是否可用(即,包含所有内角点)
            RGB_update_flag = 0;
            IR_update_flag = 0;
            Depth_update_flag = 0;

            cv::Size patternsize(7, 6); // 标定板的角点个数
            vector<cv::Point2f> corners;
            
            bool RGB_OK = findChessboardCorners(RGB_img, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
            bool IR_OK = findChessboardCorners(IR_img, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
            
            cout << RGB_OK << endl;
            cout << IR_OK << endl;

            if (RGB_OK && IR_OK)
            {
                IR_OK = 0;
                RGB_OK = 0;
                static int image_index = 1;
                char text[100];
                sprintf(text, save_dir, "RGB", image_index);
                cout << text << endl;
                // 将图像写入到text地址的文件里面
                imwrite(text, RGB_img);
                sprintf(text, save_dir, "Depth", image_index);
                imwrite(text, Depth_img);
                sprintf(text, save_dir, "IR", image_index);
                imwrite(text, IR_img);
                RCLCPP_INFO(client->get_logger(), "save %d done", image_index);
                image_index++;
            }            
        }
    } 
    //5. 资源释放
    rclcpp::shutdown();
    return 0;
}
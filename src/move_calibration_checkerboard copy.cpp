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

// 定义opencv数据格式
cv::Mat RGB_img, IR_img, Depth_img;
// 定义一些flag
bool RGB_update_flag = 0, IR_update_flag = 0, Depth_update_flag = 0;

// 订阅话题消息名称
const string topic_ir = "/camera/camera_r/custom_image";
const string topic_rgb = "/camera/camera_l/custom_image";
const string topic_depth = "/camera/camera_r/custom_image_depth";
const char *save_dir = "./src/camera_calibration/save_checkboard_img/%s/%d.png";

// 3. 创建订阅者回调函数，将传感器发送的消息转化为opencv兼容的cvimage（或者创建订阅者类）
// 创建订阅者订阅rgb
class SubImage_RGB: public rclcpp::Node
{
public:
    SubImage_RGB():Node("subimage_rgb_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建RGB订阅者");
        sub_ = this->create_subscription<Image>(
            topic_rgb, 
            10, 
            std::bind(&SubImage_RGB::Callback_rgb, this, _1)
        );
    }
private:
    rclcpp::Subscription<Image>::SharedPtr sub_;
    void Callback_rgb(const Image::ConstSharedPtr &msg)
    {
        RGB_update_flag = 0;
        try
        {
            RGB_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            RGB_update_flag = 1;
            RCLCPP_INFO(this->get_logger(), "时间戳为：%d", msg->header.stamp.nanosec);
            /* 可选择是否显示图片 */
            // cv::imshow("rgb_img", RGB_img);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调rgb出错");
            // ROS_ERROR("rgb Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
};

// 创建订阅者订阅ir
class SubImage_IR: public rclcpp::Node
{
public:
    SubImage_IR():Node("subimage_ir_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建IR订阅者");
        sub_ = this->create_subscription<Image>(
            topic_ir, 
            10, 
            std::bind(&SubImage_IR::Callback_ir, this, _1)
        );
    }
private:
    rclcpp::Subscription<Image>::SharedPtr sub_;
    void Callback_ir(const Image::ConstSharedPtr &msg)
    {
        IR_update_flag = 0;
        try
        {
            IR_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
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
    }
};

// 创建订阅者订阅depth
class SubImage_Depth: public rclcpp::Node
{
public:
    SubImage_Depth():Node("subimage_depth_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建订阅者");
        sub_ = this->create_subscription<Image>(
            topic_depth, 
            10, 
            std::bind(&SubImage_Depth::Callback_depth, this, std::placeholders::_1)
        );
    }
private:
    rclcpp::Subscription<Image>::SharedPtr sub_;
    void Callback_depth(const Image::ConstSharedPtr &msg)
    {
        Depth_update_flag = 0;
        try
        {
            cv::Mat Depth_img_32FC1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); //得到的是32FC1的图片 对应type为5
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
};

// 创建客户端
class SetCheckerState: public rclcpp::Node
{
public:
    // 2.1 利用构造函数创建客户端对象
    SetCheckerState():Node("setcheckerstate_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "创建客户端节点");
        client_ = this->create_client<SetEntityState>("set_entity_state");
        // RCLCPP_INFO(this->get_logger(), "成功创建客户端节点");
    }
    // 2.2 定义：等待连接服务端函数
    bool connect_server(){
        // wait_for_service 通过时间来判断，超时返回false，链接上返回true
        while (!client_->wait_for_service(2s))
        {
            // 当前实现存在问题：当按下ctrl c会进入死循环   
            // 1. 如何判断ctrl+c是否按下？ 
            // 2. 如何处理?
            if(!rclcpp::ok()){ //rclcpp::ok 判断程序是否正常执行，如果未正常执行（按下ctrl + c）,会返回false，正常则返回true
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中");
        }
        return true;
    }
    // 2.3 定义：发送服务请求函数
    rclcpp::Client<SetEntityState>::FutureAndRequestId send_request(string name, Pose pose, Twist twist, string reference_frame){
        // string name, Pose pose, Twist twist, string reference_frame
        // SetEntityState::Request::SharedPtr &request
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

// 回调函数
// void Callback_rgb(const Image::ConstSharedPtr &msg)
// // Image::ConstPtr &msg)
// {
//     RGB_update_flag = 0;
//     try
//     {
//         // auto tracked_object = std::shared_ptr<const void>{};
//         /*
//             ccv_bridge::CvImageConstPtr toCvShare(
//                 const sensor_msgs::msg::Image &, 
//                 const std::shared_ptr<const void> &, 
//                 const std::string & = std::string())
//         */
//         RGB_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//         RGB_update_flag = 1;
//         /* 可选择是否显示图片 */
//         // cv::imshow("rgb_img", RGB_img);
//         // cv::waitKey(1);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调rgb出错");
//         // ROS_ERROR("rgb Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//     }
// }
// void Callback_ir(const Image::ConstSharedPtr &msg)
// {
//     IR_update_flag = 0;
//     try
//     {
//         // IR_img是前面定义过的 cv::Mat
//         IR_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//         // IR_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
//         IR_update_flag = 1;
//         /* 可选择是否显示图片 */
//         // cv::imshow("ir_img", IR_img);
//         // cv::waitKey(1);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调ir出错");
//         // RCLCPP_ERROR("ir Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//     }
// }
// void Callback_depth(const Image::ConstSharedPtr &msg)
// {
//     Depth_update_flag = 0;
//     try
//     {
//         cv::Mat Depth_img_32FC1 = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone(); //得到的是32FC1的图片 对应type为5
//         Depth_img_32FC1.convertTo(Depth_img, CV_16U, 65535.0 / 1.0, 0.0);
//         Depth_update_flag = 1;
//         /* 可选择是否显示图片 */
//         // cv::imshow("Depth_img", Depth_img);
//         // cv::waitKey(1);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "回调depth出错");
//         // ROS_ERROR("depth Could not convert from depth_registered_Callback.");
//     }
// }

int main(int argc, char const *argv[])
{
    // 初始化ROS2客户端
    rclcpp::init(argc,argv);
    // 4.1 创建客户端实例化对象
    auto client = std::make_shared<SetCheckerState>();
    // 等待和gazebo服务端连接，并判断是否连接成功
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "尝试连接服务器");
    bool flag = client->connect_server();
    if (!flag){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接失败");
        return 0;
    }else{
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "服务器连接成功");
    }

    // 4.2 创建订阅者，订阅图像消息
    /*
        1. 原程序在这里使用了image_transport::ImageTransport类的方法来订阅数据,这个类传入了一个节点指针
        2. 但是程序在执行的时候产生错误：Segmentation fault，通过gdb调试程序中断在289行，但是错误原因还并没找到，如果能够解决请一定告诉我，谢谢；
    */

    // rclcpp::Node::SharedPtr node;
    // image_transport::ImageTransport It(node);
    // image_transport::Subscriber sub_rgb = It.subscribe(topic_rgb, 1, Callback_rgb);
    // image_transport::Subscriber sub_ir = It.subscribe(topic_ir, 1, Callback_ir);
    // image_transport::Subscriber sub_depth = It.subscribe(topic_depth, 1, Callback_depth);
    
    // 创建一个静态单线程节点执行器
    // rclcpp::executors::SingleThreadedExecutor exectuor;
    // exectuor.add_node(node->get_node_base_interface()); // 传入的应该是节点对象，而不是指针
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "阶段5完成");

    /*
        还有一种方法就是自己写一个订阅者类函数;
    */
    auto sub_rgb = std::make_shared<SubImage_RGB>();
    auto sub_ir = std::make_shared<SubImage_IR>();
    auto sub_depth = std::make_shared<SubImage_Depth>();
    rclcpp::executors::SingleThreadedExecutor exectuor;
    exectuor.add_node(sub_rgb);
    exectuor.add_node(sub_ir);
    exectuor.add_node(sub_depth);

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
    
    double x, y, z;
    double qx, qy, qz, qw;

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
        
        // 每spin_once，处理一个回调函数，程序运行时候可以看到执行多个spin_once后，flag全部置1,此时程序将进入if判断
        // 至于到底执行多个spin_once，这取决于回调的逻辑；
        /*
            这里有一个bug：
                1. spin_once 执行已经准备就绪的回调中的第一个回调
                2. 由于相机不停的发布，队列中持续的进入回调
                3. 出现的问题是：存入图像数据的时间戳不一致；（可以查看save_checkboard_img文件夹内 RGB IR Depth同一名称的图像，标定版位置是不一样的）
                4. 如果标定双目相机外参，需要保证左右相机拍得是同一个位置的标定板图像；
                5. 但是标定单目相机内参，不需要保证左右相机拍得是同一个位置的标定板图像；
            暂且先这样，后续会修改；
        */
        rate.sleep();   // spin_once 不阻断程序向后执行
        exectuor.spin_once();

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
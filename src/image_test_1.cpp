#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

#include<SemiGlobalMatching.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle n;

    // 创建一个发布图像消息的发布者
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("image_topic", 10);

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";

    // 读取图片
    cv::Mat img = cv::imread("/home/maxi/SGBM_ws/src/t265_to_mavros/img/test.jpg");


    // SGM匹配参数设计
    SemiGlobalMatching::SGMOption sgm_option;
    // 聚合路径数
    sgm_option.num_paths = 8;
    // 候选视差范围
    sgm_option.min_disparity = 0;
    sgm_option.max_disparity = 128;

    // census窗口类型
    sgm_option.census_size = SemiGlobalMatching::Census5x5;
    // 一致性检查
    sgm_option.is_check_lr = true;
    sgm_option.lrcheck_thres = 1.0f;
    // 唯一性约束
    sgm_option.is_check_unique = true;
    sgm_option.uniqueness_ratio = 0.99;
    // 剔除小连通区
    sgm_option.is_remove_speckles = true;
    sgm_option.min_speckle_aera = 50;
    // 惩罚项P1、P2
    sgm_option.p1 = 10;
    sgm_option.p2_init = 150;
    // 视差图填充
    sgm_option.is_fill_holes = false;

    // 定义SGM匹配类实例
    SemiGlobalMatching sgm;


    // 转换图片格式为ROS消息
    cv_image.image = img;
    sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();

    // 发布图像消息
    ros::Rate loop_rate(1);  // 设置发布频率为1Hz
    while (ros::ok())
    {
        image_pub.publish(img_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


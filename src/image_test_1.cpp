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
    //cv_image.encoding = "bgr8";
    

    // 读取图片
    cv::Mat img = cv::imread("/home/maxi/SGBM_ws/src/t265_to_mavros/img/test.jpg");

   cv::Mat img_left = cv::imread("/home/maxi/SGBM_ws/src/t265_to_mavros/img/d435i/infra1_image.png",cv::IMREAD_GRAYSCALE);
   cv::Mat img_right = cv::imread("/home/maxi/SGBM_ws/src/t265_to_mavros/img/d435i/infra2_image.png",cv::IMREAD_GRAYSCALE);

   if (img_left.data == nullptr || img_right.data == nullptr) {
        std::cout << "fail to read images" << std::endl;
        return -1;
   }
   if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
        std::cout << "img_left.size not equals img_right" << std::endl;
        return -1;
   }

   const sint32 width = static_cast<uint32>(img_left.cols);
   const sint32 height = static_cast<uint32>(img_right.rows);

   //the graydata of the left and right image
   auto bytes_left = new uint8[width * height];
   auto bytes_right = new uint8[width * height];
   for (int i = 0; i < height; i++) {
       for (int j = 0; j < width; j++) {
            bytes_left[i * width + j] = img_left.at<uint8>(i, j);
            bytes_right[i * width + j] = img_right.at<uint8>(i, j);
       }
   }


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

    // 初始化
    if (!sgm.Initialize(width, height, sgm_option)) {
        std::cout << "SGM初始化失败！" << std::endl;
        return -2;
    }
    
    // 匹配
    auto disparity = new float32[uint32(width * height)]();
    if (!sgm.Match(bytes_left, bytes_right, disparity)) {
        std::cout << "SGM匹配失败！" << std::endl;
        return -2;
    }

    // 显示视差图
    cv::Mat disp_mat = cv::Mat(height, width, CV_8UC1);
    float min_disp = width, max_disp = -width;
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            const float32 disp = disparity[i * width + j];
            if (disp == Invalid_Float) {
                disp_mat.data[i * width + j] = 0;
            }
            else {
                disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
            }
        }
    }


    // 转换图片格式为ROS消息
    //cv_image.image = img;
    cv_image.image = disp_mat;
    cv_image.encoding = "mono8";  //灰度图这里注意用mono8
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


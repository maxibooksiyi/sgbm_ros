#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

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


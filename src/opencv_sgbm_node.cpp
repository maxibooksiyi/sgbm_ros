#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <chrono>


cv::Mat img_left = cv::Mat(480, 640, CV_8UC1);

cv::Mat img_right = cv::Mat(480, 640, CV_8UC1);

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K);
void disp2Depth_(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K);

void left_gray_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        //cv::Mat left_gray_image = cv_ptr->image;
        img_left = cv_ptr->image;

        // 在这里可以对灰度图像进行处理
        // 比如显示图像信息
        ROS_INFO("Received a %d x %d grayscale image", img_left.cols, img_left.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void right_gray_imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        //cv::Mat right_gray_image = cv_ptr->image;
        img_right = cv_ptr->image;

        // 在这里可以对灰度图像进行处理
        // 比如显示图像信息
        ROS_INFO("Received a %d x %d grayscale image", img_right.cols, img_right.rows);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle n;

    // 创建一个发布图像消息的发布者
    ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("sgbm_depth_image", 10);
    // 订阅灰度图像话题
    ros::Subscriber left_gray_sub = n.subscribe("/camera/infra1/image_rect_raw", 1, left_gray_imageCallback);
    ros::Subscriber right_gray_sub = n.subscribe("/camera/infra2/image_rect_raw", 1, right_gray_imageCallback);

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;


// 将视差图转换为深度图
    double baseline = 5.01066446304321;  // 基线长度（单位：cm）
    double focal_length = 389.365356445312;  // 焦距（单位：像素）

   double fx_1 = 442.05185684172238325;
   double fy_1 = 442.05185684172238325;
   double u_1 = 158.16831588745117188;
   double v_1 = 216.55125045776367188;

   cv::Mat K1 = (cv::Mat_<double>(3,3)<< fx_1, 0, u_1, 0, fy_1, v_1, 0, 0, 1);
    // cv::Mat depth_map = baseline * focal_length / disparity_map;


	  auto start = std::chrono::high_resolution_clock::now();

    // 定义SGBM参数
    int minDisparity = 0;  // 最小视差
    int numDisparities = 16 *6 ;  // 视差范围的数量
    int blockSize = 16;  // 匹配块的大小
    int P1 = 8 * img_left.channels() * blockSize * blockSize;  // P1参数
    int P2 = 32 * img_right.channels() * blockSize * blockSize;  // P2参数
    int disp12MaxDiff = 2;  // 左右视图一致性检查的最大差异

    // 创建SGBM对象并设置参数
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize);
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setDisp12MaxDiff(disp12MaxDiff);

    ros::Rate loop_rate(10);  // 设置发布频率为1Hz
    while (ros::ok())
    {

    /******************************************************************************
    // 计算视差图
    //cv::Mat disp_mat =  cv::Mat(480, 640, CV_8UC1);;
    cv::Mat disp_mat, disp8_mat;
    //stereoSGBM->compute(imgLeft, imgRight, disp);


    int numberOfDisparities = ((640 / 8) + 15) & -16;
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
    sgbm->setPreFilterCap(32);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = img_left.channels();
    sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);

    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    ros::Rate loop_rate(10);  // 设置发布频率为1Hz
    while (ros::ok())
    {

   // 获取当前时间
   // auto start = std::chrono::high_resolution_clock::now();

    // 创建一个CvBridge对象
    cv_bridge::CvImage cv_image;

    // 获取当前时间
    auto start = std::chrono::high_resolution_clock::now();

    sgbm->compute(img_left, img_right, disp_mat);

    // 获取结束时间
    auto end = std::chrono::high_resolution_clock::now();
 
    // 计算耗时时间，单位为毫秒
    std::chrono::duration<double, std::milli> duration = end - start;
 
    // 输出耗时时间
    std::cout << "代码执行耗时: " << duration.count() << " 毫秒" << std::endl;   

    disp_mat.convertTo(disp8_mat, CV_8U, 255/(numberOfDisparities*16.));

    //cv::Mat disp_guiyihua_mat =  cv::Mat(480, 640, CV_8UC1);;
    //normalize(disp_mat, disp_guiyihua_mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  //我自己加上深度图的生成
    //left camera paras
   double fx_1 = 442.05185684172238325;
   double fy_1 = 442.05185684172238325;
   double u_1 = 158.16831588745117188;
   double v_1 = 216.55125045776367188;

   cv::Mat K1 = (cv::Mat_<double>(3,3)<< fx_1, 0, u_1, 0, fy_1, v_1, 0, 0, 1);

   //cv::Mat depth_mat = cv::Mat(height, width, CV_8UC1);
   cv::Mat depth_mat = cv::Mat(480, 640, CV_16UC1);  //深度图看了下都是16UC1，包括D435i的深度图
   //cv::Mat depth_mat;
   disp2Depth(disp8_mat, depth_mat, K1);
   //disp2Depth(disp_guiyihua_mat, depth_mat, K1);

   /***
   // 获取结束时间
    auto end = std::chrono::high_resolution_clock::now();

    // 计算耗时时间，单位为毫秒
    std::chrono::duration<double, std::milli> duration = end - start;

    // 输出耗时时间
    std::cout << "代码执行耗时: " << duration.count() << " 毫秒" << std::endl;
    ***/

    //*********************************************************************************/


    // 计算视差图像
    cv::Mat disparity;
    sgbm->compute(img_left, img_right, disparity);

    // 将视差图像归一化到0-255范围内
    //cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

    //cv::Mat depth_mat = cv::Mat(480, 640, CV_16UC1);  //深度图看了下都是16UC1，包括D435i的深度图
    cv::Mat depth_mat = cv::Mat(480, 640, CV_16UC1, cv::Scalar(0));  //深度图看了下都是16UC1，包括D435i的深度图
    //disp2Depth(disparity, depth_mat, K1);
    disp2Depth_(disparity, depth_mat, K1);
    //depth_mat = baseline * focal_length /(disparity);

    // 转换图片格式为ROS消息
    //cv_image.image = img;
    //cv_image.image = disp_mat;
    cv_image.image = depth_mat;
    //cv_image.encoding = "mono8";  //灰度图这里注意用mono8
    //cv_image.encoding = "mono16";
    cv_image.encoding = "16UC1";
    sensor_msgs::ImagePtr img_msg = cv_image.toImageMsg();

        image_pub.publish(img_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}




/*
函数作用：视差图转深度图
参考自 https://blog.csdn.net/qq_36955294/article/details/84140788  
输入：
　　dispMap ----视差图，8位单通道，CV_8UC1
　　K       ----内参矩阵，float类型
输出：
　　depthMap ----深度图，16位无符号单通道，CV_16UC1
*/

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
{
    int type = dispMap.type();

    //float fx = K.at<float>(0, 0);  //这么读取的fx的值变为了2.03019e+24
    double fx = K.at<double>(0, 0);  //现在找到之前自动读取fx值不对的原因了，double类型的变量用float类型去读取导致的
    //float fx = 422;
    //float fx = 382; //d435i
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    //float baseline = 6500; //基线距离65mm
    //float baseline = 80.0/camera_scale;
    //float baseline = 50; //D435i的双目基线是5厘米
    float baseline = 50;
    //cout << "maxitest0" << endl;
    if (type == CV_8U)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;

        //uchar* dispData = (uchar*)dispMap.data;
        uint8_t* dispData = (uint8_t*)dispMap.data;
        ushort* depthData = (ushort*)depthMap.data;  //ushort关键字是System.UInt16的别名 
        for (int i = 0; i < height; i++)
        {
            //cout << "maxitest1" << endl;
            for (int j = 0; j < width; j++)
            {
                //cout << "maxitest2" << endl;
                int id = i*width + j;
                //cout << "id is" << id << endl;
                //cout << "像素height坐标i is" << i << endl;
                //cout << "像素width坐标j is" << j << endl;
                if (!dispData[id])  continue;  //防止0除
                //depthData[id] = ushort( (float)fx *baseline / ((float)dispData[id]) );
                //depthData[id] =  fx *baseline / ((int)dispMap.data[id]) ;
                //depthMap.data[id] = ushort( fx *baseline / ((int)dispMap.data[id]) );
                depthMap.at<uint16_t>(i, j) = ushort( fx *baseline / ((int)dispMap.data[id]) );
                //cout << "fx为" << fx << endl;
                //cout << "baseline为" << baseline << endl;
                //cout << "dispData[id]为" << dispData[id] << endl;
                //cout << "dispMap.data[id]为" << (int)dispMap.data[id] << endl;
                //cout << "算出的像素值也就是深度值(单位mm)为" << depthData[id] << endl;
                //cout << "算出的像素值也就是深度值(单位mm)为" << (uint16_t)depthMap.data[id] << endl;
                //std::cout << "算出的像素值也就是深度值(单位mm)为" << (uint16_t)depthMap.at<uint16_t>(i, j) << std::endl;
            }
        }
    }
    else
    {
        std::cout << "please confirm dispImg's type!" << std::endl;
        //cv::waitKey(0);
    }
}



void disp2Depth_(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K)
{
    int type = dispMap.type();

    //float fx = K.at<float>(0, 0);  //这么读取的fx的值变为了2.03019e+24
    double fx = K.at<double>(0, 0);  //现在找到之前自动读取fx值不对的原因了，double类型的变量用float类型去读取导致的
    //float fx = 422;
    //float fx = 382; //d435i
    float fy = K.at<float>(1, 1);
    float cx = K.at<float>(0, 2);
    float cy = K.at<float>(1, 2);
    //float baseline = 6500; //基线距离65mm
    //float baseline = 80.0/camera_scale;
    //float baseline = 50; //D435i的双目基线是5厘米
    float baseline = 50;
    //cout << "maxitest0" << endl;
    //if (type == CV_8U)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;

        //uchar* dispData = (uchar*)dispMap.data;
        //uint8_t* dispData = (uint8_t*)dispMap.data;
        ushort* dispData = (ushort*)dispMap.data;
        ushort* depthData = (ushort*)depthMap.data;  //ushort关键字是System.UInt16的别名 
        for (int i = 0; i < height; i++)
        {
            //cout << "maxitest1" << endl;
            for (int j = 0; j < width; j++)
            {
                //cout << "maxitest2" << endl;
                int id = i*width + j;
                //cout << "id is" << id << endl;
                //cout << "像素height坐标i is" << i << endl;
                //cout << "像素width坐标j is" << j << endl;
                //if (!dispData[id])  continue;  //防止0除
                if (!dispMap.at<uint16_t>(i, j))  continue;  //防止0除
                //depthData[id] = ushort( (float)fx *baseline / ((float)dispData[id]) );
                //depthData[id] =  fx *baseline / ((int)dispMap.data[id]) ;
                //depthMap.data[id] = ushort( fx *baseline / ((int)dispMap.data[id]) );
                depthMap.at<uint16_t>(i, j) = ushort( fx *baseline / (((uint16_t)dispMap.at<uint16_t>(i, j))/16) );
                //cout << "fx为" << fx << endl;
                //cout << "baseline为" << baseline << endl;
                //cout << "dispData[id]为" << dispData[id] << endl;
                //cout << "dispMap.data[id]为" << (int)dispMap.data[id] << endl;
                //cout << "算出的像素值也就是深度值(单位mm)为" << depthData[id] << endl;
                //cout << "算出的像素值也就是深度值(单位mm)为" << (uint16_t)depthMap.data[id] << endl;
                //std::cout << "算出的像素值也就是深度值(单位mm)为" << (uint16_t)depthMap.at<uint16_t>(i, j) << std::endl;
            }
        }
    }
}

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

#include<SemiGlobalMatching.h>

void disp2Depth(cv::Mat dispMap, cv::Mat &depthMap, cv::Mat K);

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


  //我自己加上深度图的生成
    //left camera paras
   double fx_1 = 442.05185684172238325;
   double fy_1 = 442.05185684172238325;
   double u_1 = 158.16831588745117188;
   double v_1 = 216.55125045776367188;

   cv::Mat K1 = (cv::Mat_<double>(3,3)<< fx_1, 0, u_1, 0, fy_1, v_1, 0, 0, 1);

   //cv::Mat depth_mat = cv::Mat(height, width, CV_8UC1);
   cv::Mat depth_mat = cv::Mat(height, width, CV_16UC1);  //深度图看了下都是16UC1，包括D435i的深度图
   //cv::Mat depth_mat;
   disp2Depth(disp_mat, depth_mat, K1);

   uint16_t max_in_depth_mat = 0;
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
               int id = i*width + j;
               //cout << "depth_mat.data " << (uint16_t)depth_mat.data[id] << endl;
               //cout << "depthMap.at<uint16_t>(i, j) " << (uint16_t)depth_mat.at<uint16_t>(i, j) << endl;
               //不能通过mat.data[id]的方式访问16位深度的深度图！！！！！它只能读取8位的值
               //printf("depth_mat.data=%d\n", depth_mat.data[id]);
               if( depth_mat.at<uint16_t>(i, j) > max_in_depth_mat )
               {
                  max_in_depth_mat = (uint16_t)depth_mat.at<uint16_t>(i, j) ;
                  //cout << "depth_mat.data " << (uint16_t)depth_mat.at<uint16_t>(i, j)  << endl;
               }

            }
        }
    //cout << "max_in_depth_mat " << max_in_depth_mat << endl;

   cv::Mat depth_mat_8U_for_show = cv::Mat(height, width, CV_8UC1);  //创建一个8位的灰度图用来显示16位的深度图
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
            {
                int id = i*width + j;
                //depthData[id] = ushort( (float)fx *baseline / ((float)dispData[id]) );
                ///depthData[id] =  fx *baseline / ((int)dispMap.data[id]) ;
                //depth_mat_8U_for_show.data[id] = ((int)depth_mat.data[id]/65535)*255;
                //depth_mat_8U_for_show.data[id] = ((int)depth_mat.data[id]/max_in_depth_mat)*255;
                depth_mat_8U_for_show.at<uint8_t>(i, j) = ( (float)depth_mat.at<uint16_t>(i, j)/max_in_depth_mat)*255;
                //cout << "depth_mat_8U_for_show.at<uint16_t>(i, j) " << (uint16_t)depth_mat_8U_for_show.at<uint16_t>(i, j) << endl;
            }
        }

    cv::imwrite("/home/maxi/SGBM_ws/src/t265_to_mavros/img/d435i/infra1_image_depth_maxi.png", depth_mat_8U_for_show);


    // 转换图片格式为ROS消息
    //cv_image.image = img;
    //cv_image.image = disp_mat;
    cv_image.image = depth_mat;
    //cv_image.encoding = "mono8";  //灰度图这里注意用mono8
    //cv_image.encoding = "mono16";
    cv_image.encoding = "16UC1";
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
    float baseline = 50; //D435i的双目基线是5厘米
    //cout << "maxitest0" << endl;
    if (type == CV_8U)
    {
        const float PI = 3.14159265358;
        int height = dispMap.rows;
        int width = dispMap.cols;

        //uchar* dispData = (uchar*)dispMap.data;
        uint8* dispData = (uint8*)dispMap.data;
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
                std::cout << "算出的像素值也就是深度值(单位mm)为" << (uint16_t)depthMap.at<uint16_t>(i, j) << std::endl;
            }
        }
    }
    else
    {
        std::cout << "please confirm dispImg's type!" << std::endl;
        //cv::waitKey(0);
    }
}

#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>

//using namespace cv;

//代码参考自：https://blog.csdn.net/qq_37648371/article/details/135005259  
//编译命令 g++ -o opencv_sgbm_demo_test.cpp opencv_sgbm_demo_test.cpp.cpp `pkg-config --cflags --libs opencv`

int main() {

    cv::Mat image_left = imread("infra1_image.png", cv::IMREAD_GRAYSCALE);
    cv::Mat image_right = imread("infra2_image.png", cv::IMREAD_GRAYSCALE);

// 将视差图转换为深度图
    double baseline = 5.01066446304321;  // 基线长度（单位：cm）
    double focal_length = 389.365356445312;  // 焦距（单位：像素）
    // cv::Mat depth_map = baseline * focal_length / disparity_map;


	  auto start = std::chrono::high_resolution_clock::now();

    // 定义SGBM参数
    int minDisparity = 0;  // 最小视差
    int numDisparities = 16 *6 ;  // 视差范围的数量
    int blockSize = 16;  // 匹配块的大小
    int P1 = 8 * image_left.channels() * blockSize * blockSize;  // P1参数
    int P2 = 32 * image_right.channels() * blockSize * blockSize;  // P2参数
    int disp12MaxDiff = 2;  // 左右视图一致性检查的最大差异

    // 创建SGBM对象并设置参数
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize);
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setDisp12MaxDiff(disp12MaxDiff);

    // 计算视差图像
    cv::Mat disparity;
    sgbm->compute(image_left, image_right, disparity);

    // 将视差图像归一化到0-255范围内
    cv::normalize(disparity, disparity, 0, 255, cv::NORM_MINMAX, CV_8U);

    cv::Mat depth_map = baseline * focal_length /(disparity);
    cv::Mat depthColor;
    cv::applyColorMap(depth_map, depthColor, cv::COLORMAP_JET);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end - start;
    double elapsedTime = duration.count();

    std::cout << "compute disparty map time cost = " << elapsedTime << " seconds. " << std::endl;

    // 显示视差图
    cv::imshow("Disparity Map", disparity);
    cv::imshow("Depth Map", depthColor);

    cv::waitKey(0);

}




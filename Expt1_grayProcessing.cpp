#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
	ROS_WARN("*****START*****");
	ros::init(argc, argv, "trafficLaneTrack"); //初始化ROS节点
	ros::NodeHandle n;

	//Before the use of camera, you can test ur program with images first: imread()
	VideoCapture capture;
	capture.open(0); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	waitKey(100);
	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}

#ifndef READIMAGE_ONLY
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel", 5); //定义dashgo机器人的速度发布器
#endif
	Mat src_frame;
	while (ros::ok())
	{
		capture.read(src_frame);
		if (src_frame.empty())
		{
			break;
		}
		imshow("src", src_frame);
		// 此处为实验部分，请自行增加直方图均衡化的代码

    const int grayMax=255;
	vector<vector<int>>graylevel(grayMax+1);
	Mat image=imread("Lpic0.jpg",0);
	Mat img;
	Mat src;
	image.copyTo(img);
	if (!image.data)
	{
		return -1;
	}
	for (int i = 0; i < image.rows-1; i++)
	{
		uchar* ptr=image.ptr<uchar>(i);
		for (int j = 0; j < image.cols-1; j++)
		{
			int x=ptr[j];
			graylevel[x].push_back(0);//这个地方写的不好，引入二维数组只是为了记录每一个灰度值的像素个数
		}
	}
	for (int i = 0; i < img.rows-1; i++)
	{
		uchar* imgptr=img.ptr<uchar>(i);
		uchar* imageptr=image.ptr<uchar>(i);
		for (int j = 0; j < img.cols-1; j++)
		{
			int sumpiexl=0;
			for (int k = 0; k < imageptr[j]; k++)
			{
				sumpiexl=graylevel[k].size()+sumpiexl;
			}
			imgptr[j]=(grayMax*sumpiexl/(image.rows*image.cols));
		}

	}
	equalizeHist(image,src);
	imshow("原图",image);
	imshow("opencv自带",src);
	imshow("自己实现",img);
	waitKey(0);

#ifndef READIMAGE_ONLY
		//以下代码可设置机器人的速度值，从而控制机器人运动
		geometry_msgs::Twist cmd_red;
		cmd_red.linear.x = 0;
		cmd_red.linear.y = 0;
		cmd_red.linear.z = 0;
		cmd_red.angular.x = 0;
		cmd_red.angular.y = 0;
		cmd_red.angular.z = 0.2;
		pub.publish(cmd_red);
#endif
		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}

#include <stdlib.h>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <string>

#include <geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define LINEAR_X 0

using namespace cv; 
using namespace std;



//滤波
//空域高斯滤波函数
void generateGaussMask(cv::Mat& Mask,cv::Size wsize, double sigma)
//void generateGaussMask(Mat Mask,cv::Size wsize, double sigma)
{
	Mask.create(wsize,CV_64F);
	int h = wsize.height;
	int w = wsize.width;
	int center_h = (h - 1) / 2;
	int center_w = (w - 1) / 2;
	double sum = 0.0;
	double x, y;
	for (int i = 0; i < h; ++i)
	{
		y = pow(i - center_h, 2);
		for (int j = 0; j < w; ++j)
		{
			x = pow(j - center_w, 2);
			double g = exp(-(x + y) / (2 * sigma*sigma));
			Mask.at<double>(i, j) = g;
			sum += g;
		}
	}
	for(int i=0;i<h;i++)
	{
		for(int j=0;j<w;j++)
		{
			Mask.at<double>(i,j)=Mask.at<double>(i,j)/sum;
		}
	}
}

void Gaussian(cv::Mat& src, cv::Mat& dst, cv::Mat Mask)
//void Gaussian(Mat src, Mat dst, Mat Mask)
{
	int hh = (Mask.rows - 1) / 2;
	int hw = (Mask.cols - 1) / 2;
	dst = cv::Mat::zeros(src.size(),src.type());
	//边界填充
	cv::Mat Newsrc;
	cv::copyMakeBorder(src, Newsrc, hh, hh, hw, hw, cv::BORDER_REPLICATE);//边界复制
	
	//高斯滤波
	for (int i = hh; i < src.rows + hh;++i)
	{
		for (int j = hw; j < src.cols + hw; ++j)
		{
			double sum[3] = { 0 };
			for (int r = -hh; r <= hh; ++r)
			{
				for (int c = -hw; c <= hw; ++c)
				{
					if (src.channels() == 1)
					{
						sum[0] = sum[0] + Newsrc.at<uchar>(i + r, j + c) * Mask.at<double>(r + hh, c + hw);
					}
					else if (src.channels() == 3)
					{
						cv::Vec3b rgb = Newsrc.at<cv::Vec3b>(i+r,j + c);
						sum[0] = sum[0] + rgb[0] * Mask.at<double>(r + hh, c + hw);//B
						sum[1] = sum[1] + rgb[1] * Mask.at<double>(r + hh, c + hw);//G
						sum[2] = sum[2] + rgb[2] * Mask.at<double>(r + hh, c + hw);//R
					}
				}
			}
 
			for (int k = 0; k < src.channels(); ++k)
			{
				if (sum[k] < 0)
					sum[k] = 0;
				else if (sum[k]>255)
					sum[k] = 255;
			}
			if (src.channels() == 1)
			{
				dst.at<uchar>(i - hh, j - hw) = static_cast<uchar>(sum[0]);
			}
			else if (src.channels() == 3)
			{
				cv::Vec3b rgb = { static_cast<uchar>(sum[0]), static_cast<uchar>(sum[1]), static_cast<uchar>(sum[2]) };
				dst.at<cv::Vec3b>(i-hh, j-hw) = rgb;
			}
 
		}
	}
 
}

/*
void Gaussian(Mat input, double sigma)
{
	//滤波器模板
	Mat Mask;
	Mask.create(cv::Size(3,3),CV_64F);	
	double sum_m=0.0;
	double x,y;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
		{
			double tmp=exp(-((i-1)*(i-1)+(j-1)*(j-1))/(2*sigma*sigma));
			sum_m+=tmp;
			Mask.at<double>(i,j)=tmp;
		}
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			Mask.at<double>(i,j)=Mask.at<double>(i,j)/sum_m;
	//高斯滤波
	//cv::Mat windows;
	//std::cout<<Mask<<std::endl;
	int r=(Mask.rows-1)/2;
	int c=(Mask.cols-1)/2;
	Mat output=cv::Mat::zeros(input.size(),input.type());
	for(int i=r;i<r+input.rows;++i)
		for(int j=c;j<c+input.cols;++j)
		{
			double sum=0;
			for(int p=-r;p<=r;++p)
				for(int q=-c;q<=c;++q)
					sum=sum+input.at<uchar>(i+p,j+q)*Mask.at<double>(p+r,q+c);
			if(sum<0)
				sum=0;
			else if (sum>255)
				sum=255;
			output.at<uchar>(i-r,j-c)=static_cast<uchar>(sum);
		}
	imshow("output",output);
	//waitKey(0);
}

void Gaussian3(Mat input, double sigma)
{
	//滤波器模板
	Mat Mask;
	Mask.create(cv::Size(3,3),CV_64F);	
	double sum_m=0.0;
	double x,y;
	for(int i=0;i<3;++i)
		for(int j=0;j<3;++j)
		{
			double tmp=exp(-((i-1)*(i-1)+(j-1)*(j-1))/(2*sigma*sigma));
			sum_m+=tmp;
			Mask.at<double>(i,j)=tmp;
		}
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			Mask.at<double>(i,j)=Mask.at<double>(i,j)/sum_m;
	//高斯滤波
	//cv::Mat windows;
	//std::cout<<Mask<<std::endl;
	int r=(Mask.rows-1)/2;
	int c=(Mask.cols-1)/2;
	Mat output=cv::Mat::zeros(input.size(),input.type());
	for(int i=r;i<r+input.rows;++i)
		for(int j=c;j<c+input.cols;++j)
		{
			double sum[3]={0};
			for(int p=-r;p<=r;++p)
				for(int q=-c;q<=c;++q)
				{
					sum[0]=sum[0]+input.at<uchar>(i+p,j+q)*Mask.at<double>(p+r,q+c);
					sum[1]=sum[1]+input.at<uchar>(i+p,j+q)*Mask.at<double>(p+r,q+c);
					sum[2]=sum[2]+input.at<uchar>(i+p,j+q)*Mask.at<double>(p+r,q+c);
				}
			//if(sum<0)
			//	sum=0;
			//else if (sum>255)
			//	sum=255;
			cv::Vec3b rgb={static_cast<uchar>(sum[0]),static_cast<uchar>(sum[1]),static_cast<uchar>(sum[2])};
			output.at<cv::Vec3b>(i-r,j-c)=rgb;
		}
	imshow("output",output);
	//waitKey(0);
}*/


// 形态学
// 膨胀函数
void Dilate(Mat Src, Mat Tem, Mat Dst)
{
	Dst.create(Src.rows,Src.cols,CV_8UC1);
	int m = (Tem.rows - 1) / 2;
	int n = (Tem.cols - 1) / 2;
	for (int i = m; i < Src.rows - m -1; i++)//i、j的范围保证结构元始终在扩展后的图像内部
	{
		for (int j = n; j < Src.cols - n -1; j++)
		{
			Mat SrcROI = Src(Rect(j - m, i - n, Tem.cols, Tem.rows));
			double sum = 0;
			sum = SrcROI.dot(Tem);//矩阵对应位置相乘后求和
			if (sum == 9)//结构元的9个元素均为1，和为9才能保证结构元完全包含于相应集合
				Dst.at<uchar>(i, j) = 255;
			else
				Dst.at<uchar>(i, j) = 0;
		}
	}
	imshow("膨胀处理", Dst);
}

// 腐蚀函数
void Erode(Mat Src, Mat Tem, Mat Dst)
{
	Dst.create(Src.rows,Src.cols,CV_8UC1);
	int m = (Tem.rows - 1) / 2;
	int n = (Tem.cols - 1) / 2;
	for (int i = m; i < Src.rows - m - 1; i++)//i、j的范围保证结构元始终在扩展后的图像内部
	{
		for (int j = n; j < Src.cols - n - 1; j++)
		{
			Mat SrcROI = Src(Rect(j - m, i - n, Tem.cols, Tem.rows));
			double sum = 0;
			sum = SrcROI.dot(Tem);//矩阵对应位置相乘后求和
			if (sum != 0)//结构元的9个元素均为1，只要和不为0，就能说明结构元与相应集合有交集
				Dst.at<uchar>(i, j) = 255;
			else
				Dst.at<uchar>(i, j) = 0;
		}
	}
	imshow("腐蚀处理", Dst);
}
/*
//膨胀函数
void Dilate(Mat img, Mat output)
{
	//Mat output;
	output.create(img.rows,img.cols,CV_8UC1);
	for(int i=0;i<img.rows-1;++i)
	{
		for(int j=0;j<img.cols-1;++j)
		{
			uchar max=0;
			uchar temp=(std::max<uchar>)(img.at<uchar>(i+1,j),img.at<uchar>(i,j));
			max=(std::max<uchar>)(temp,img.at<uchar>(i,j+1));
			output.at<uchar>(i,j)=max;
		}
	}
	imshow("膨胀处理",output);
	//waitKey(0);
	
}

//腐蚀函数
void Erode(Mat img, Mat output)
{
	//Mat output;
	output.create(img.rows,img.cols,CV_8UC1);
	for(int i=2;i<img.rows-2;++i)
	{
		for(int j=2;j<img.cols-2;++j)
		{

			//for(int p=i-1;p<=i+1;p++)
			//{
			//	for(int q=j-1;q<=j+1;q++)
			//	{
			//		if(p<0||q<0||q>=img.cols||p>=img.rows)
			//			continue;
			//		max=(std::max<uchar>)(max,img.at<uchar>(i,j));
			//	}
			//}
			//output.at<uchar>(i,j)=max;
			//output.at<uchar>(i+1,j)=max;
			//output.at<uchar>(i,j+1)=max;
			uchar min=0;
			uchar temp=(std::min<uchar>)(img.at<uchar>(i+1,j),img.at<uchar>(i,j));
			min=(std::min<uchar>)(temp,img.at<uchar>(i,j+1));
			output.at<uchar>(i,j)=min;
		}
	}
	imshow("腐蚀处理",output);
	//waitKey(0);
	
}*/


int main(int argc, char **argv)
{

	VideoCapture capture;
	capture.open(0);//1为打开zed相机，0为打开笔记本摄像头

	ROS_WARN("*****START");
	ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
		ros::NodeHandle n;
		
		// ros::Rate loop_rate(10);//定义速度发布频率
		ros::Publisher pub= n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel",5);//定义速度发布器
		
	if (!capture.isOpened())
    {
        cout<<"cannot open the camera!"<<endl;
        return 0;
    }
	waitKey(1000);
	Mat frame;//当前帧图片
	Mat dst1;
	Mat dst2;
	Mat dst3;
	Mat gray;
	Mat Mask;
	Mat binary;
	int nFrames= 0;//图片帧数
	int frameWidth= capture.get(CV_CAP_PROP_FRAME_WIDTH);//图片宽
	int frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);//图片高
	
	
	while(ros::ok())
	{
		capture.read(frame);
		if(frame.empty())
			break;

		Mat frln= frame.clone();//使用笔记本摄像头
		//Mat frln=frame(cv::Rect(0,0,frame.cols/2,frame.rows));//截取zed左目图片

		Mat mTem(3, 3, CV_8UC1, Scalar(1));

		//	空域高斯滤波函数
		//generateGaussMask(Mask, cv::Size(3, 3), 0.8);
		//Gaussian(frln, dst1, Mask);
		imshow("原图", frln);
		
		// 膨胀函数
		cvtColor(frln, gray, COLOR_BGR2GRAY);
		imshow("灰度", gray);
		//Gaussian(gray, 1);
		threshold(gray, binary, 200, 255, CV_THRESH_BINARY);
		imshow("二值", binary);
		
		generateGaussMask(Mask, cv::Size(3, 3), 3);
		Gaussian(gray, dst1, Mask);
		imshow("高斯滤波处理",dst1);

		//Dilate(binary);
		Dilate(binary, mTem, dst2);
		//imshow("膨胀处理",dst2);
		// 腐蚀函数
		Erode(binary, mTem, dst3);
		//imshow("腐蚀处理",dst3);
		//Erode(binary);
		/*for (int i = 1; i < nImages; i++)
      	{
       	 	loadImage(frln,i);
       	 	imshow("原图",frln);
			loadImage(dst1,i);
       	 	imshow("高斯滤波处理",dst1);
			loadImage(dst1,i);
       	 	imshow("灰度图",gray);
			loadImage(dst2,i);
			imshow("膨胀处理",dst2);
			loadImage(dst3,i);
			imshow("腐蚀处理",dst3);

       		waitKey(100000);

      }*/
		//imshow("原图",frln);
		//imshow("高斯滤波处理",dst1);
		//imshow("灰度图",gray);
		//imshow("膨胀处理",dst2);
		//imshow("腐蚀处理",dst3);
		
		ros::spinOnce();
		waitKey(5);
	}
	return 0;
}

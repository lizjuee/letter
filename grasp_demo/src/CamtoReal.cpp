/*该接口是实现像素坐标映射到三维坐标。即通过物体识别得到的像素坐标来获取物体距离相机的实际坐标xyz(CamtoReal.cpp)。*/
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <grasp_demo/cam2real.h>

using namespace cv;
using namespace std;

class ImageConverter
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_depth; //接收深度图像
	image_transport::Subscriber image_sub_color; //接收彩色图像

	ros::Subscriber camera_info_sub_; //接收深度图像对应的相机参数话题

	ros::ServiceServer CamtoReal;
	double pixel_x, pixel_y;

	sensor_msgs::CameraInfo camera_info;
	geometry_msgs::PointStamped output_point;

	/* Mat depthImage,colorImage; */
	Mat colorImage;
	Mat depthImage = Mat::zeros(480, 640, CV_16UC1); //camera_info

public:
	ImageConverter() : it_(nh_)
	{
		//topic sub:
		image_sub_depth = it_.subscribe("/cam_1/aligned_depth_to_color/image_raw",
										1, &ImageConverter::imageDepthCb, this);

		image_sub_color = it_.subscribe("/cam_1/color/image_raw", 1,
										&ImageConverter::imageColorCb, this);
		camera_info_sub_ =
			nh_.subscribe("/cam_1/aligned_depth_to_color/camera_info", 1,
						  &ImageConverter::cameraInfoCb, this);

		CamtoReal = nh_.advertiseService("/cam_to_real", &ImageConverter::CamtoRealCallback, this);
	}

	~ImageConverter()
	{
	}

	void cameraInfoCb(const sensor_msgs::CameraInfo &msg)
	{
		camera_info = msg;
	}

	void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try
		{
			cv_ptr =
				cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			depthImage = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	void imageColorCb(const sensor_msgs::ImageConstPtr &msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			colorImage = cv_ptr->image;
		}
		catch (cv_bridge::Exception &e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

	bool CamtoRealCallback(grasp_demo::cam2real::Request &req,
						   grasp_demo::cam2real::Response &res)
	{
		pixel_x = req.pixel_x;
		pixel_y = req.pixel_y;
		float real_z = 0.001 * depthImage.at<u_int16_t>(pixel_y, pixel_x);
		float real_x =
			(pixel_x - camera_info.K.at(2)) / camera_info.K.at(0) * real_z;
		float real_y =
			(pixel_y - camera_info.K.at(5)) / camera_info.K.at(4) * real_z;
		if (real_z != 0)
		{
			res.obj_x = real_x;
			res.obj_y = real_y;
			res.obj_z = real_z;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cam_to_real");
	ImageConverter imageconverter;
	ros::spin();
	return (0);
}

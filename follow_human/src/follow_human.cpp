#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>

static const std::string OPENCV_IMAGE = "Image window";

class ImageConverter {
	ros::NodeHandle nHandle;
	image_transport::ImageTransport imageTprt;
	image_transport::Subscriber subscriber;
	image_transport::Publisher publisher;
	image_transport::Subscriber depthSubscriber;
	ros::Publisher velocityPub = nHandle.advertise < geometry_msgs::Twist > ("/cmd_vel_mux/input/navi", 10);
	float angular, linear;
	int human_row, human_col;
	const int bot_x, bot_y, bot_z; 
	ros::Subscriber sub = nHandle.subscribe("odom", 1000, chatterCallback);

public:
	ImageConverter() :
	imageTprt(nHandle) {
		subscriber = imageTprt.subscribe("/usb_cam/image_raw", 10, &ImageConverter::imageDistance, this);
		
		
		subscriberSpencer = imageTprt.subscribe("/spencer/perception/tracker_persons", 10, &ImageConverter::imageDistance, this);
		depthSubscriber = imageTprt.subscribe("/camera/depth/image_raw", 10, &ImageConverter::depthDistance, this);
		publisher = imageTprt.advertise("/image_converter/output_video", 10);
		cv::namedWindow (OPENCV_IMAGE);
	}
	~ImageConverter() {
		cv::destroyWindow (OPENCV_IMAGE);
	}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
 	ROS_INFO("Seq: [%d]", msg->header.seq);
   ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
   bot_x = msg->pose.pose.position.x;
   bot_y = msg->pose.pose.position.y;
   bot_z = msg->pose.pose.position.z;
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
   ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}
	void chase() {
		float linearx = 1.6;
		float angulary = 1.6;
		geometry_msgs::Twist velocityMsg;
		velocityMsg.linear.y = velocityMsg.linear.z = 0;
		velocityMsg.angular.x = velocityMsg.angular.y = 0;
		velocityMsg.linear.x = linearx * linear;
		velocityMsg.angular.z = angulary * angular;
		velocityPub.publish(velocityMsg);
	}

	void imageDistance(const sensor_msgs::ImageConstPtr& msg) {
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg,
					sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		int row = 0;
		int col = 0;
		int dis_min = 1000000;
		for (int i = 0; i < cv_ptr->image.rows; i++) {
			for (int j = 0; j < cv_ptr->image.cols; j++) {
				int b = cv_ptr->image.at < cv::Vec3b > (i, j).val[0];
				int g = cv_ptr->image.at < cv::Vec3b > (i, j).val[1];
				int r = cv_ptr->image.at < cv::Vec3b > (i, j).val[2];
				int dis = r * r + g * g + b * b;
				int bot_dis = bot_x * bot_y * bot_z;
				dis = dis - bot_dis; 
				if (dis < dis_min) {
					dis_min = dis;
					row = i;
					col = j;
				}
			}
		}
		human_row = row;
		human_col = col;
		if (human_col < msg->width * 0.45)
			angular = -1.0 * (human_col - 300) / 300.0;
		else if (human_col > msg->width * 0.55)
			angular = -1.0 * (human_col - 300) / 300.0;
		else
			;
		chase();
		cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255, 0, 0));
		// Update GUI
		cv::imshow(OPENCV_IMAGE, cv_ptr->image);
		cv::waitKey(3);
		// Output latest graphics
		publisher.publish(cv_ptr->toImageMsg());
	}

	void depthDistance(const sensor_msgs::Image::ConstPtr& msg) {
		//Distance from ball
		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			float distance = cv_ptr->image.at<float>(human_row, human_col);
			ROS_INFO("Distance: %f", distance);
			if (distance < 999 && distance > 0)
				linear = (distance - 1000) / 8000;
			else if (distance > 1100)
				linear = (distance - 1000) / 8000.0;
			else if (distance == 0.00)
				linear = linear;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "follow_human_node");
	
	ImageConverter ic;
	ros::spin();
	return 0;
}


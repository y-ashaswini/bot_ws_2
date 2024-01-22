#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include "image_transport/image_transport.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


void handleTwist(const geometry_msgs::Twist& msg);
void handleCamera(const sensor_msgs::ImageConstPtr& msg);

std::string OPENCV_WINDOW = "Camera";

int main(int argc, char** argv)
{
    ros::init(argc,argv,"coldetect_node");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Subscriber twist = nh.subscribe("cmd", 1000, handleTwist);
    image_transport::Subscriber camera = it.subscribe("camera/depth/image_rect_raw", 1, handleCamera);
    cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_AUTOSIZE);

    ros::spin();

    cv::destroyAllWindows();
    return 0;

}

void handleTwist(const geometry_msgs::Twist& msg){
    ROS_INFO("\nlinear:{%.1f,%.1f,%.1f}\nangular:{%.1f,%.1f,%.1f}", msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);    
}

void handleCamera(const sensor_msgs::ImageConstPtr& msg){
    //ROS_INFO("Image received");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      ROS_INFO("copied image");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(100);
}
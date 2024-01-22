#include "ros/ros.h"
#include "publisher_subscriber/MsgTutorial.h"

// function is called when a topic message named "ros_tutorial_msg" is recieved.
// as an input message, MsgTutorial message of the ros_tutorials_topic package is recieved

void msgCallback(const publisher_subscriber::MsgTutorial::ConstPtr& msg)
{
	ROS_INFO("recieved sec%d", msg->stamp.sec);
	ROS_INFO("recieved nsec %d", msg->stamp.nsec);
	ROS_INFO("recieved data %d", msg->data);	
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "topic_subscriber");
	ros::NodeHandle nh;
	ros::Subscriber subscriber = nh.subscribe("message", 100, msgCallback);

	// A function for calling a callback function, waiting for a message to be received, and executing a callback function when it is received

	ros::spin();
	return 0;
}


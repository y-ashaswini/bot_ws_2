#include "ros/ros.h"
#include "publisher_subscriber/MsgTutorial.h"

int main(int argc, char **argv) {
	ros::init(argc, argv, "topic_publisher");
	ros::NodeHandle nh;
	ros::Publisher publisher = nh.advertise<publisher_subscriber::MsgTutorial>("message", 100);
	ros::Rate loop_rate(10);
	publisher_subscriber::MsgTutorial msg;	
	int count = 0;
	
	while(ros::ok()) {
		msg.stamp = ros::Time::now();
		msg.data = count;
		
		ROS_INFO("sending message at%d", msg.stamp.sec);
		ROS_INFO("sending message at %d", msg.stamp.nsec);;
		ROS_INFO("sending message data %d", msg.data);
		
		publisher.publish(msg);
		loop_rate.sleep();
		
		++count;
	}
	return 0;

}

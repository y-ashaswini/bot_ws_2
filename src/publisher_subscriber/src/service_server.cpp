#include "ros/ros.h"
#include "publisher_subscriber/SrvFile.h"

bool calculations(publisher_subscriber::SrvFile::Request &req, publisher_subscriber::SrvFile::Response &res) {
    res.result = req.a+req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int) req.b);
    ROS_INFO("sending back response: %ld", (long int)res.result);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "service_server");
    ros::NodeHandle nh;

    ros::ServiceServer service_server_node = nh.advertiseService("service_server_node", calculations);

    ROS_INFO("ready svr server!");
    ros::spin(); // spin: wait until response

    return 0;   
}
#include "ros/ros.h"
#include "publisher_subscriber/SrvFile.h"
#include <cstdlib> // for using the "atoll" function

int main(int argc, char **argv)
{
    ros::init(argc, argv, "service_client");
    if (argc != 3)
    {
        ROS_INFO("cmd : rosrun publisher_subscriber service_client arg0 arg1");
        ROS_INFO("arg0: double number, arg1: double number");
        return 1;
    }

    ros::NodeHandle nh;

    ros::ServiceClient service_client_node =
        nh.serviceClient<publisher_subscriber::SrvFile>("service_client");
    publisher_subscriber::SrvFile srv;
    // Parameters entered when the node is executed as a service request value are stored at 'a' and 'b'
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    // Request the service.
    // If the request is accepted, display the response value

    if (service_client_node.call(srv))
    {
        ROS_INFO("send srv, srv.Request.a and b: %ld, %ld", (long int)srv.request.a, (long int)srv.request.b);
        ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);
    }
    else
    {
        ROS_WARN("Failed to call service service_server");
        return 1;
    }
    return 0;
}
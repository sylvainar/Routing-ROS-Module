#include "ros/ros.h"
#include "routing_machine/RoutingMachine.h"


bool test(routing_machine::RoutingMachine::Request  &req, routing_machine::RoutingMachine::Response &res)
{
	
  for(int i = 0; i < 4; i++)
  {
  	/*
  	res.latitude[i] = (float)i;
  	res.longitude[i] = (float)(i+1);
  	*/
  	res.latitude.push_back ((float)i);
  	res.longitude.push_back((float)i);
  }

  //ROS_INFO("request: x=%f, y=%f", (float)req.start_latitude, (float)req.start_longitude);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "routing_machine");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("routing_machine", test);
  ROS_INFO("Routing Machine ready !");
  ros::spin();

  return 0;
}
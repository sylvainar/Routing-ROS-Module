#include "ros/ros.h"
#include "routing_machine/RoutingMachine.h"


bool test(routing_machine::RoutingMachine::Request  &req, routing_machine::RoutingMachine::Response &res)
{
	
  for(int i = 0; i < 4; i++)
  {
  	res.coords.push_back((float)i*10); //Latitude 
  	res.coords.push_back((float)i*100); //Longitude
  }

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

#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Int16.h"


int main(int argc, char ** argv)
{
	std::cout<<"Initializing create path..."<<std::endl;
	ros::NodeHandle n;
	ros::Publisher pubPath = n.advertise<nav_msgs::Path>("/navigation/goal_path",1);
	ros::Rate loop(10);
	nav_msgs::Path goalpath;
	while(ros::ok())
	{
		/*Crear y publicar el path a seguir*/
		pubPath.publish(goalpath);
		ros::spinOnce();
		loop.sleep();	
	}
}

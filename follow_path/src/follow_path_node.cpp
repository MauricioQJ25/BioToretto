#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "nav_msgs/Path.h"

std_msgs::Int16 speedP;
std_msgs::Int16 steeringP;

void callback_errorcalc(const nav_msgs::Path::ConstPtr& msg)
{
        std::cout<<"Leyendo Path"<<std::endl;
        /*Funcion que calcula el error*/
        /*Funcion que calcula las nuevas velocidades y steering*/
        speedP.data = 3000;
        steeringP.data = 290;

}

int main(int argc,char **  argv)
{
        std::cout << "Inicializando sigue el camino..." <<std::endl;
        ros::init(argc,argv,"followPath");
        ros::NodeHandle n;
        ros::Publisher pubSpeed = n.advertise<std_msgs::Int16>("/manual_control/speed" ,1);
        ros::Publisher pubSteering = n.advertise<std_msgs::Int16>("/manual_control/steering",1);
        ros::Subscriber subPath = n.subscribe("/lane",1,callback_errorcalc);
        ros::Rate loop(10);

        while(ros::ok())
        {
                pubSpeed.publish(speedP);
                pubSteering.publish(steeringP);
                ros::spinOnce();
                loop.sleep();
        }

        return 0;
}


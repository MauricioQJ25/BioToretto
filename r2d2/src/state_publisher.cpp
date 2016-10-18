#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>


double Vx  = 0;
double Vy  = 0;
double Vth = 0;
//Variables del auto
double alpha=0;
double theta_punto=0;
double radio =0.034;
double dist_llantas=0.26;
//Variables intermedias para conversion steering
double ang_max=0.57;//angulo en radianes max
double ang_max_int=170;//290 - 120 considerando 120 como salida a 0 radianes
double ang_zero_int=120;
//Variables intermedias para conversion speed
double vmax=1;//metros por segundo
double vmax_int=4000;

//*****************************************************************************************
//float speed=.1;
void callbackSpeed(const std_msgs::Int16 speed)
{
//     ROS_INFO("I heard Vel: linear[%lf]",msg);
     //evaluar las velocidades máximas alcanzadas por el coche 
     if (speed.data < 4000 && speed.data > -4000 ){
        theta_punto =double(speed.data);

        Vx = 2*M_PI*radio*theta_punto*(vmax/(2*M_PI*radio*vmax_int));

        Vy  = 0;
     }
     else{
      Vx=vmax;
     }
      
   
}

void callbackSteering(const std_msgs::Int16 steering)
{
  // ROS_INFO("I heard Vel: steering[%lf]",msg1);
   //evaluar el steering máximo alcanzado por el coche 
   if(steering.data<290 && steering.data > -50){
      alpha = (double(steering.data)-120)*(ang_max/ang_max_int);
      Vth = Vx*sin(alpha)/dist_llantas;
   }else 
   if(steering.data <= -50){
      Vth= Vx*sin(-ang_max)/dist_llantas;  
   }else
   if(steering.data >=290){
      Vth= Vx*sin(ang_max)/dist_llantas;
   }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    tf::TransformBroadcaster broadcaster;
    
    ros::Subscriber velAngular_sub = n.subscribe("/manual_control/steering",1,callbackSteering);
    ros::Subscriber vel_sub = n.subscribe("/manual_control/speed", 1, callbackSpeed);

    // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    double x  = 0.0;
    double y  = 0.0;
    double th = 0.0;
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::Rate r(30);

    //const double degree = M_PI/180;

    
    //double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    

    while (ros::ok()) {
        ros::spinOnce();               // check for incoming messages
        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x  = (Vx * cos(th) - Vy * sin(th)) * dt;
        double delta_y  = (Vx * sin(th) + Vy * cos(th)) * dt;
        double delta_th = Vth * dt;

        x  += delta_x;
        y  += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation      = odom_quat;

        //send the odometry transform
        broadcaster.sendTransform(odom_trans);

        //now send the transform from base_link to camera_link
        broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
                current_time,"base_link", "camera_link"));

        //Now create the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x  = x;
        odom.pose.pose.position.y  = y;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x  = Vx;
        odom.twist.twist.linear.y  = Vy;
        odom.twist.twist.angular.z = Vth;

        //publish the odometry message
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }
}

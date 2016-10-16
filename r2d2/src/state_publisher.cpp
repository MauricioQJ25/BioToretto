#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>


double Vx  = 0;
double Vy  = 0;
double Vth = 0;
//Variables del auto
double alpha=0;
double theta_punto=0;
double radio =0.03;
double dist_llantas=0.38;

//*****************************************************************************************
//float speed=.1;
void callbackSpeed(const geometry_msgs::Twist::ConstPtr& msg)
{
     ROS_INFO("I heard Vel: linear[%lf %lf %lf]  steering[%lf %lf %lf]",
           msg->linear.x, msg->linear.y, msg->linear.z,
           msg->angular.x, msg->angular.y, msg->angular.z);

  theta_punto = msg->linear.x;
  alpha = msg->angular.z;

  Vx = 2*M_PI*radio*theta_punto;
  Vth = Vx*sin(alpha)/dist_llantas;

  Vy  = msg->linear.y;
   
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
    tf::TransformBroadcaster broadcaster;

    ros::Subscriber vel_sub = n.subscribe("/hardware/speed", 1, callbackSpeed);

    // ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    double x  = 0.0;
    double y  = 0.0;
    double th = 0.0;
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    
    ros::Rate r(30);

    const double degree = M_PI/180;

    
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    

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

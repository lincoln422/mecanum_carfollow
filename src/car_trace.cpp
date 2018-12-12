#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//轨迹
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <math.h>
#include <tf/tf.h>

#define speed 0.5
#define Rwr 0.05
#define l3 0.22
#define l4 0.20
void motor_to_speed(double& m1, double& m2, double& m3, double& m4,double & theta ,geometry_msgs::Twist& msg)
{
//电机速度转角速度线速度；zheng运动学
  /*
double k = 190.98609;  //600/PI
msg.linear.x = (m1+m2)/(2*k) ;
msg.linear.y = (m3-m2)/(2*k) ;
msg.angular.z = (m4-m2)/(0.84*k) ;
*/
 msg.linear.x = Rwr/4 * ( cos(theta)*(m1+m2+m3+m4)  - sin(theta)*(-m1+m2+m3-m4) )  ;
 msg.linear.y = Rwr/4 * ( sin(theta)*(m1+m2+m3+m4)  + cos(theta)*(-m1+m2+m3-m4) )  ;
 msg.angular.z = Rwr/4 * (1/(l3+l4) ) *(-m1+m2-m3+m4);
  
}

 double posex,posey,poseth;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "caronetrace_publisher");
  ros::NodeHandle nh;
  geometry_msgs::Twist test_msg;
  nav_msgs::Path path;
  
  //设置初始值
  test_msg.linear.x = 0;
  test_msg.linear.y = 0;
  test_msg.linear.z = 0;
  test_msg.angular.x = 0;
  test_msg.angular.y = 0;
  test_msg.angular.z = 0;
  
  double w11,w21,w31,w41;  //线速度 0s~45s or  45s~90s
  double w12,w22,w32,w42;  
  /*
  w11 = speed * 4; 
  w21 = speed * 6; 
  w31 = speed * 4.8; 
  w41 = speed * 5.2;
  w12 = speed * 6; 
  w22 = speed * 4; 
  w32 = speed * 5.2; 
  w42 = speed * 4.8;
  */
  
  w11 = speed * 4; 
  w21 = speed * 4; 
  w31 = speed * 4; 
  w41 = speed * 4;
  w12 = speed * 4; 
  w22 = speed * 4; 
  w32 = speed * 4; 
  w42 = speed * 4;
  
  /*
  w11 = speed * 240; 
  w21 = speed * 360; 
  w31 = speed * 288; 
  w41 = speed * 312;
  w12 = speed * 360; 
  w22 = speed * 240; 
  w32 = speed * 312; 
  w42 = speed * 288;
  */
 
  posex = posey = poseth = 0;
  
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();              //init times
  last_time = ros::Time::now();
  ros::Time fortyfive_seconds = ros::Time::now() + ros::Duration(45);  //45s
  ros::Time ninty_seconds = ros::Time::now() + ros::Duration(90);  //90s
  
  path.header.stamp=current_time;
  path.header.frame_id="odom";
  
  ros::Rate loop_rate(15);  //HZ
  ros::Publisher chatter_pub = nh.advertise<geometry_msgs::Twist>("car1speed",1); //发布话题对象初始化
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);

  //0~45s
  while(ros::ok() && (ros::Time::now()<=fortyfive_seconds) ){
    
    motor_to_speed(w11,w21,w31,w41,poseth,test_msg);    //vel transfrom
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x =  (test_msg.linear.x * cos(poseth) - test_msg.linear.y * sin(poseth)) * dt;
    double delta_y =  (test_msg.linear.x * sin(poseth) + test_msg.linear.y * cos(poseth)) * dt;
    double delta_th =  test_msg.angular.z * dt;
    posex += delta_x;
    posey += delta_y;
    poseth += delta_th;
    
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = posex;
        this_pose_stamped.pose.position.y = posey;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseth);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);
    
    path_pub.publish(path);
    chatter_pub.publish(test_msg);
    
   // ROS_INFO(" \nlinear.x:%f;\n linear.y:%f;\n angular.z:%f;\n ",
	  //   test_msg.linear.x,test_msg.linear.y,test_msg.angular.z);
    
    last_time = current_time;
    loop_rate.sleep();
    
  }
  
  //45s-90s
  while(ros::ok() && (ros::Time::now()>fortyfive_seconds && ros::Time::now()<=ninty_seconds ) ) {
    
    motor_to_speed(w12,w22,w32,w42,poseth,test_msg);
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (test_msg.linear.x * cos(poseth) - test_msg.linear.y * sin(poseth)) * dt;
    double delta_y = (test_msg.linear.x * sin(poseth) + test_msg.linear.y * cos(poseth)) * dt;
    double delta_th = test_msg.angular.z * dt;
    posex += delta_x;
    posey += delta_y;
    poseth += delta_th;
    
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = posex;
        this_pose_stamped.pose.position.y = posey;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseth);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);
    
    path_pub.publish(path);
    chatter_pub.publish(test_msg);
    
    //ROS_INFO(" \nlinear.x:%f;\n linear.y:%f;\n angular.z:%f;\n ",
	   //  test_msg.linear.x,test_msg.linear.y,test_msg.angular.z);
    
    last_time = current_time;
    loop_rate.sleep();
    
  }

  return 0;
}

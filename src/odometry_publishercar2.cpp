#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#define speed 1

class OdometryPublisher
{
public:
  OdometryPublisher(std::string model_link_name, std::string model_vel_topic_name, double init_pos_x, double init_pos_y, double init_pos_th);
  
private:
  void velMessageRsceived(const geometry_msgs::Twist &msg);
  void motor_to_speed(double &m1, double &m2, double &m3, double &m4, geometry_msgs::Twist& msg); 
  ros::NodeHandle nh;
  ros::Subscriber odom_sub;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Time current_time, last_time;
  double x, y, th;
  double vx, vy, vth;
  double v1, v2, v3, v4;

  std::string model_link;
  std::string model_vel_topic;
};



OdometryPublisher::OdometryPublisher(std::string model_link_name, std::string model_vel_topic_name, double init_pos_x, double init_pos_y, double init_pos_th)
{

  model_link = model_link_name;
  model_vel_topic = model_vel_topic_name;

  x = init_pos_x;
  y = init_pos_y;
  th = init_pos_th;
  v1 = v2 = v3 = v4 = 0;

  odom_pub = nh.advertise<nav_msgs::Odometry>("odomcar2", 50);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  odom_sub = nh.subscribe(model_vel_topic, 10, &OdometryPublisher::velMessageRsceived, this);
}

void OdometryPublisher::velMessageRsceived(const geometry_msgs::Twist &msg)
{
  // v1 = msg.linear.x;
  // v2 = msg.linear.y;
  // v3 = msg.linear.z;
  // v4 = msg.angular.z;

  // vx = speed * (v4 + v1 + v2 - v3) / 4;
  // vy = speed * (v3 + v4 - v1 + v2) / 4;
  // vth = speed * (+v2 - v4 + v1 + v3) / 1.68;

  vx = speed * msg.linear.x;
  vy = speed * msg.linear.y;
  vth = speed * msg.angular.z;

  current_time = ros::Time::now();

  //compute odometry in a typical way given the velocities of the robot

  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;

  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odomcar2";
  odom_trans.child_frame_id = model_link;
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odomcar2";

  //set the position

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = model_link;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

  //publish the message
  odom_pub.publish(odom);

  last_time = current_time;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::AsyncSpinner spinner(1);
  
  spinner.start();
  ros::Rate loop_rate(15);
 // OdometryPublisher car1_odom("/car1/base_footprint", "car1speed", 0, 0, 0);
  OdometryPublisher car2_odom("/car2/base_footprint", "car2speed", -0.25, -0.25, 0);
  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

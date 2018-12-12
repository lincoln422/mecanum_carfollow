#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <nav_msgs/Path.h>
#define Rw 0.08
#define Rwr 0.05
#define N 3
#define Nu 2
#define l1 0.32
#define l2 0.30
#define l3 0.22
#define l4 0.20
#define restric 0.2
#define speedddd 0.5

using namespace Eigen;
using namespace std;


using namespace Eigen;
using namespace std;

  double aaa, bbb, ccc, ddd;
  double u1, u2, u3;
  double m_;
  
  double period_;
  double gamma_; 
  double w_follow, vx_follow, vy_follow;
  double w1_follow, w2_follow, w3_follow, w4_follow;
  double posex_follow, posey_follow, poseth_follow;
  double posex_follow_path, posey_follow_path, poseth_follow_path;
  MatrixXd Q_(6, 6);
  MatrixXd R_(9, 9);
  Eigen::Matrix<double,6,6> tildrI_;
  Eigen::Matrix<double,3,3> gxx_;
  Eigen::Matrix<double,3,3> z_;
  Eigen::Matrix<double,9,6> G_; 
  Eigen::Matrix<double,6,6> W_;
  VectorXd abarUmin_(6);
  VectorXd abarUmax_(6);
  VectorXd abarxmin_(9);
  VectorXd abarxmax_(9);
  VectorXd detabarUmin_(6);
  VectorXd detabarUmax_(6);
  MatrixXd E_;
  MatrixXd M_;
  MatrixXd ImH_;
  MatrixXd IpHt_;
  VectorXd Pinfty_;
  VectorXd Minfty_;
  VectorXd yinput_;
  VectorXd p_(36);
  geometry_msgs::Twist msgkkk;
  geometry_msgs::Twist msglast;
  geometry_msgs::Twist msgojbk;
   // changing-all-the-time values
  VectorXd fxx_(3);
  VectorXd xk_(3);
  VectorXd tildrf_(9);
  VectorXd tildrg_(9);
  VectorXd u_(3);
  VectorXd C_(6);
  VectorXd b_(30);
  MatrixXd M_follow_(3, 3);
  MatrixXd theta_3x3follow_(3, 3);
  MatrixXd theta_3x4follow_(3, 4);
  MatrixXd theta_3x4forward_(3, 4);
  VectorXd D(3);
  VectorXd Dx(9);
  VectorXd abaru(6);
  double linearx, lineary, angularw;
  geometry_msgs::Quaternion msg_temp;
  geometry_msgs::Quaternion msg_temp2;
  double w1, w2, w3, w4;
  double roll, pitch, yaw;
   double roll2, pitch2, yaw2;
    double car1x, car1y, theta;
  tf::Quaternion quat;
   tf::Quaternion quat2;
   
   double xr_x, yr_y, thr_th;
   nav_msgs::Path path2;


class Follow
{
private:
  ros::NodeHandle nh;
  ros::Subscriber car1_sub;
  ros::Subscriber car2_sub;
  //ros::Subscriber car1pose_sub;
  ros::Publisher car2_pub;
  ros::Publisher path_pub ;

  //void count_u(const nav_msgs::OdometryConstPtr &car1pose);
  void car1_sub_Cb(const nav_msgs::OdometryConstPtr &poseandspeed);
  void car2_sub_Cb(const nav_msgs::OdometryConstPtr &poseandspeed2);
  //电机速度转角速度线速度；
  //void motor_to_speed(double &m1, double &m2, double &m3, double &m4,double &theta, geometry_msgs::Twist& msg);
  //void motor_to_speed(double &m1,double &m2, double &m3 ,double &m4,double& lx, double& ly, double& az);
                                                         //oid speed_to_motor(double& m1, double& m2, double& m3, double& m4,double &theta, geometry_msgs::Twist &speed);
  //void speed_to_motor(double& m1, double& m2, double& m3, double& m4, double& lx, double& ly, double& az );
    //电机速度转角速度线速度；
    void motor_to_speed_BIG(double &m1, double &m2, double &m3, double &m4,double &theta, geometry_msgs::Twist& msg);
 // void motor_to_speed_BIG(double &m1,double &m2, double &m3 ,double &m4,double& lx, double& ly, double& az);
 // void speed_to_motor_BIG(double& m1, double& m2, double& m3, double& m4, geometry_msgs::Twist &speed);
 // void speed_to_motor_BIG(double& m1, double& m2, double& m3, double& m4, double& lx, double& ly, double& az );
  
  VectorXd gfuntion(Eigen::VectorXd u);
  VectorXd runge_kuttaul();
  VectorXd weifenfangcheng(VectorXd yinput_);
  //void count_c_d(const geometry_msgs::TwistConstPtr &car2speed); 

public:
  Follow();
  
  
protected:


};

Follow::Follow()
{

  //car1_sub = nh.subscribe<geometry_msgs::Twist>("odom", 1, &Follow::car1_sub_Cb, this);
  car2_pub = nh.advertise<geometry_msgs::Twist>("car2speed", 1);
  car1_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &Follow::car1_sub_Cb, this);
   car2_sub = nh.subscribe<nav_msgs::Odometry>("odomcar2", 1, &Follow::car2_sub_Cb, this);
  //car2_sub = nh.subscribe<geometry_msgs::Twist>("car2speed", 1, &Follow::count_c_d, this);
  //car1pose_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, &Follow::count_u, this);
   path_pub  = nh.advertise<nav_msgs::Path>("trajectory2", 1, true);
                
   
}

Eigen::VectorXd Follow::gfuntion(Eigen::VectorXd u)
{
  VectorXd yp_(36);
  VectorXd ym_(36);
  yp_ << detabarUmax_, Pinfty_;                             // 36x1
  ym_ << detabarUmin_, Minfty_;                             // 36x1
  VectorXd temp(36);
  temp = VectorXd::Zero(36);
  for (int i = 0;i != 36;i++ ) {
    if (u[i] > yp_[i])
      {
      temp[i] = yp_[i];
      }
      else if (u[i] < ym_[i])
      {
      temp[i] = ym_[i];
      }
      else
      {
      temp[i] = u[i];
      }
  }
  //ROS_INFO("gfuntion is ok");
  return temp;
}

Eigen::VectorXd Follow::weifenfangcheng(Eigen::VectorXd yinput_)
{

VectorXd udotz(36);
udotz=gamma_*IpHt_*(gfuntion(ImH_*yinput_-p_)-yinput_);
//ROS_INFO("weifenfangcheng is ok");
//std::cout<< "udotz is" << std::endl<< udotz << std::endl;
return udotz;
}


Eigen::VectorXd Follow::runge_kuttaul()
{
  	double h=0.01;
	VectorXd k11(36);
	VectorXd k21(36);
	VectorXd k31(36);
	VectorXd k41(36);
	int n=10;
	for(int i=0;i<n;i++){
		k11=weifenfangcheng(yinput_);
		k21=weifenfangcheng(yinput_+h*k11/2);
		k31=weifenfangcheng(yinput_+h*k21/2);
		k41=weifenfangcheng(yinput_+h*k31);
		yinput_=yinput_+h*(k11+2*k21+2*k31+k41)/6;
	}
	//ROS_INFO("longgekuta OK");
	//std::cout<< "yinput_ is" << std::endl<< yinput_ << std::endl;
	return yinput_;
	
}


//电机速度转角速度线速度；逆运动学
/*
void Follow::motor_to_speed(double &m1,double &m2, double &m3 ,double &m4,double &theta, geometry_msgs::Twist& msg){
//double k = 3.18310155;
double k = 190.98609;
//600/PI190.98609
msg.linear.x = (m1+m2)/(2*k) ;
msg.linear.y = (m3-m2)/(2*k) ;
msg.angular.z = (m4-m2)/(0.84*k) ;

 msg.linear.x = Rwr/4 * ( cos(theta)*(m1+m2+m3+m4)  - sin(theta)*(-m1+m2+m3-m4) )  ;
 msg.linear.y = Rwr/4 * ( sin(theta)*(m1+m2+m3+m4)  + cos(theta)*(-m1+m2+m3-m4) )  ;
 msg.angular.z = Rwr/4 * (1/(l3+l4) ) *(-m1+m2-m3+m4);

ROS_INFO(" motor to speed version 1 OK");
ROS_INFO("msg.linear.x%f", msg.linear.x);
ROS_INFO("msg.linear.y%f", msg.linear.y);
ROS_INFO("msg.angular.z%f", msg.angular.z);

}
*/

/*
void Follow::motor_to_speed(double &m1,double &m2, double &m3 ,double &m4,double& lx, double& ly, double& az){
  double k = 190.98609;
//double k = 3.18310155;                                     //600/PI
lx = (m1+m2)/(2*k) ;
ly = (m3-m2)/(2*k) ;
az = (m4-m2)/(0.84*k) ;
ROS_INFO(" motor to speed version 2 OK");
ROS_INFO("msg.linear.x%f", lx);
ROS_INFO("msg.linear.y%f", ly);
ROS_INFO("msg.angular.z%f", az);
}
*/

/*
void Follow::speed_to_motor(double& m1, double& m2, double& m3, double& m4,geometry_msgs::Twist& speed )
{ double k = 190.98609;
  //double k = 3.18310155;
  m1 = k*( speed.linear.x + speed.linear.y +0.42 * speed.angular.z );
  m2 = k*( speed.linear.x - speed.linear.y -0.42 * speed.angular.z );
  m3 = k*( speed.linear.x + speed.linear.y -0.42 * speed.angular.z );
  m4 = k*( speed.linear.x - speed.linear.y +0.42 * speed.angular.z );
 // ROS_INFO("speed to motor version 1 OK");
 // ROS_INFO("motor m1 is %f", m1);
 // ROS_INFO("motor m2 is %f", m2);
  //ROS_INFO("motor m3 is %f", m3);
  //ROS_INFO("motor m4 is %f", m4);
}
*/
/*
void Follow::speed_to_motor(double& m1, double& m2, double& m3, double& m4, double& lx, double& ly, double& az)
{
  double k = 190.98609;
//double k = 3.18310155;
  m1 = k*( lx + ly +0.42 * az );
  m2 = k*( lx - ly -0.42 * az );
  m3 = k*( lx + ly -0.42 * az );
  m4 = k*( lx - ly +0.42 * az );
  ROS_INFO("speed to motor version 2 OK");
  ROS_INFO("motor m1 is %f", m1);
  ROS_INFO("motor m2 is %f", m2);
  ROS_INFO("motor m3 is %f", m3);
  ROS_INFO("motor m4 is %f", m4);
}
*/

void Follow::motor_to_speed_BIG(double &m1,double &m2, double &m3 ,double &m4,double &theta, geometry_msgs::Twist& msg){
 
 
 //double k = 305.577749;
//double k = 5.09296248;                                     //600/PI
//msg.linear.x = (m1+m2)/(2*k) ;
//msg.linear.y = (m3-m2)/(2*k) ;
//msg.angular.z = (m4-m2)/(1.24*k) ;
 msg.linear.x = Rw/4 * ( cos(theta)*(m1+m2+m3+m4)  - sin(theta)*(-m1+m2+m3-m4) )  ;
 msg.linear.y = Rw/4 * ( sin(theta)*(m1+m2+m3+m4)  + cos(theta)*(-m1+m2+m3-m4) )  ;
 msg.angular.z = Rw/4 * (1/(l1+l2) ) *(-m1+m2-m3+m4);

//ROS_INFO(" motor to speed_BIG version 1 OK");
//ROS_INFO("msg.linear.x is %f", msg.linear.x);
//ROS_INFO("msg.linear.y is %f", msg.linear.y);
//ROS_INFO("msg.angular.z is %f", msg.angular.z);
}

/*
void Follow::motor_to_speed_BIG(double &m1,double &m2, double &m3 ,double &m4,double& lx, double& ly, double& az){
  double k = 305.577749;
//double k = 5.09296248;                                     //600/PI
lx = (m1+m2)/(2*k) ;
ly = (m3-m2)/(2*k) ;
az = (m4-m2)/(1.24*k) ;
//ROS_INFO(" motor to speed_BIG version 2 OK");
//ROS_INFO("msg.linear.x is %f", lx);
//ROS_INFO("msg.linear.y is %f", ly);
//ROS_INFO("msg.angular.z is %f", az);
}

*/

/*
void Follow::speed_to_motor_BIG(double& m1, double& m2, double& m3, double& m4, geometry_msgs::Twist& speed )
{ 
  double k = 305.577749;
  //double k = 5.09296248;
  m1 = k*( speed.linear.x + speed.linear.y +0.62 * speed.angular.z );
  m2 = k*( speed.linear.x - speed.linear.y -0.62 * speed.angular.z );
  m3 = k*( speed.linear.x + speed.linear.y -0.62 * speed.angular.z );
  m4 = k*( speed.linear.x - speed.linear.y +0.62 * speed.angular.z );
  //ROS_INFO("speed to motor_BIG version 1 OK");
  //ROS_INFO("motor m1 is %f", m1);
  //ROS_INFO("motor m2 is %f", m2);
  //ROS_INFO("motor m3 is %f", m3);
  //ROS_INFO("motor m4 is %f", m4);
}

*/
/*
void Follow::speed_to_motor_BIG(double& m1, double& m2, double& m3, double& m4, double& lx, double& ly, double& az)
{
   //double k = 5.09296248;
   double k = 305.577749;
  m1 = k*( lx + ly +0.62 * az );
  m2 = k*( lx - ly -0.62 * az );
  m3 = k*( lx + ly -0.62 * az );
  m4 = k*( lx - ly +0.62 * az);
  //ROS_INFO("speed to motor_BIG version 2 OK");
  //ROS_INFO("motor m1 is %f", m1);
  //ROS_INFO("motor m2 is %f", m2);
  //ROS_INFO("motor m3 is %f", m3);
  //ROS_INFO("motor m4 is %f", m4);
}
*/
void Follow::car2_sub_Cb(const nav_msgs::OdometryConstPtr& poseandspeed2)
{
  
  posex_follow = poseandspeed2->pose.pose.position.x;       // backward one

  posey_follow = poseandspeed2->pose.pose.position.y;
   
  msg_temp2.w = poseandspeed2->pose.pose.orientation.w;
  msg_temp2.x = poseandspeed2->pose.pose.orientation.x;
  msg_temp2.y = poseandspeed2->pose.pose.orientation.y;
  msg_temp2.z = poseandspeed2->pose.pose.orientation.z;
   
     tf::quaternionMsgToTF(msg_temp2, quat2);
      //double roll, pitch, yaw;
      tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2);
  poseth_follow = yaw2; 
     ROS_INFO("callback fuction 2 OK");
    // ROS_INFO("posex_follow is %f", posex_follow);
   //  ROS_INFO("posey_follow is %f", posey_follow);
   //  ROS_INFO("poseth_follow is %f", poseth_follow);
  //theta_3x3follow_ << cos(poseth_follow), -sin(poseth_follow), 0, sin(poseth_follow), cos(poseth_follow), 0, 0, 0, 1;
  //VectorXd xiansudu(3);
  //VectorXd dianjisudu(4);
  //dianjisudu << w1_follow, w2_follow, w3_follow, w4_follow;
  //xiansudu = Rw/4 * theta_3x3follow_ * theta_3x4follow_ * dianjisudu; 
   // ROS_INFO("vx is %f", xiansudu[0]);
    //ROS_INFO("vy is %f", xiansudu[1]);
   // ROS_INFO("anz is %f", xiansudu[2]);
  //msgkkk.linear.x = xiansudu[0];
  //msgkkk.linear.y = xiansudu[1];
  //msgkkk.angular.z = xiansudu[2];
  //msgkkk.linear.z = 0;
  //msgkkk.angular.x = 0;
  //msgkkk.angular.y = 0;
  
}



 void Follow::car1_sub_Cb(const nav_msgs::OdometryConstPtr& poseandspeed)
{
  
    

  
  //geometry_msgs::Quaternion msg_temp;
  //std::cout<< std::endl<<"pose is: " << std::endl<<  poseandspeed->pose.pose<< std::endl;
 // std::cout<< std::endl<<"twist is: " << std::endl<<  poseandspeed->twist.twist<< std::endl;
 
  msg_temp.w = poseandspeed->pose.pose.orientation.w;
  msg_temp.x = poseandspeed->pose.pose.orientation.x;
  msg_temp.y = poseandspeed->pose.pose.orientation.y;
  msg_temp.z = poseandspeed->pose.pose.orientation.z;
  
  //nav_msgs::Odometry tempodommsg;
  
  //tf::Quaternion quat;
  tf::quaternionMsgToTF(msg_temp, quat);
      //double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  car1x = poseandspeed->pose.pose.position.x;
  car1y = poseandspeed->pose.pose.position.y;
  theta = yaw; 
  
                                             // get theta ???=
                                            
  // ROS_INFO("yaw is %f", yaw);
  ROS_INFO("x is %f", car1x );  
  ROS_INFO("y is %f", car1y );  
  ROS_INFO("theta is %f", theta);                                   
 // u1 =  (Rwr * cos( theta ) * aaa - Rw * ccc)/4 ;
//  u2 = ( (-Rwr) * cos( theta) * bbb + Rw * ddd)/4; 
//  u3 =  poseandspeed->twist.twist.angular.z - w_follow;

// ROS_INFO("u1 is %f", u1); 
// ROS_INFO("u2 is %f", u2); 
// ROS_INFO("u3 is %f", u3); 
  
  linearx = poseandspeed->twist.twist.linear.x;
  lineary = poseandspeed->twist.twist.linear.y;             
  angularw = poseandspeed->twist.twist.angular.z;
                         // 6x1
  //ROS_INFO("linearx_forward is %f", linearx); 
  //ROS_INFO("lineary_forward is %f", lineary); 
  //ROS_INFO("angularw_forward is %f", angularw); 
 // std::cout <<"abaru is"<< abaru << std::endl;
  
  //xk_ << car1speed->linear.x, car1speed->linear.y, car1speed->angular.z; //3x1
  xr_x = car1x - posex_follow;
  yr_y = car1y - posey_follow;
  thr_th = theta - poseth_follow;
  ROS_INFO("xr_x is %f", xr_x); 
  ROS_INFO("yr_y is %f", yr_y); 
  ROS_INFO("thr_th is %f", thr_th);
  
  xk_ << ( cos(poseth_follow) * xr_x + sin(poseth_follow) *yr_y ) , (cos(poseth_follow) *yr_y-sin(poseth_follow) *xr_x), thr_th;
  //xk_ << -(car1x - posex_follow), -(car1y - posey_follow ), -(theta - poseth_follow);
  //double w1, w2, w3, w4;
  
  //speed_to_motor(w1, w2, w3, w4, linearx, lineary, angularw);
  
  
  aaa = w1+w2+w3+w4;
  bbb = w1-w2-w3+w4;
  //double ww =  w_follow ;
  //std::cout << xk_;
  //ROS_INFO("aaa is %f", aaa); 
  //ROS_INFO("bbb is %f", bbb); 
  
  
  M_follow_ <<0, w_follow, (Rwr * bbb )/4, -w_follow, 0, (Rwr * aaa)/4, 0, 0, 0;   // 3x3
  //std::cout <<"M_follow_ is" <<  M_follow_<< std::endl ;

  fxx_ = xk_ + period_ * M_follow_ * xk_;                   // 3x1
  tildrf_ << fxx_, fxx_, fxx_;                              // 9x1
  u_ << u1, u2, u3;                                         // 3x1
  
  //std::cout<< std::endl<< "fxx is" << std::endl<< fxx_ << std::endl ;
  tildrg_ << gxx_*u_, gxx_*u_, gxx_*u_;                     // 9x1
  //std::cout<< std::endl<< "tildrg_ is" << std::endl<< tildrg_ << std::endl ;
  //std::cout<< std::endl<< "tildrf_ is" << std::endl<< tildrf_ << std::endl ;
  //std::cout<< std::endl<< "tildrI_ is" << std::endl<< tildrI_ << std::endl ;
  //std::cout<< std::endl<< "W_ is" << std::endl<< W_ << std::endl ;
  W_ << 2 * G_.transpose() * Q_ * G_;                       // 6x6s
  
  C_ <<2* G_.transpose() * Q_ *(tildrg_ + tildrf_ - Dx);    // 6x1
   //std::cout<< std::endl << "C_ is" << std::endl<< C_ << std::endl ;
  b_ << (-abarUmin_ + abaru), (abarUmax_ -abaru),  (-abarxmin_ + tildrf_ + tildrg_ ), (abarxmax_ - tildrf_ - tildrg_);
  //std::cout<< std::endl<< "b_  is" << std::endl<< b_  << std::endl ;
  
  // 30x1
  p_ << C_, -b_;
  // std::cout<< std::endl<< "p_  is" << std::endl<< p_ << std::endl ;
    
  //runge_kuttaul();                                         // 36x1
  yinput_ = runge_kuttaul();
  //std::cout<< std::endl<< "yinput_ is" << std::endl<< yinput_ << std::endl ;
  u1 = yinput_[0]; u2 = yinput_[1]; u3 = yinput_[2];
  
  if (u1 >= restric)
    u1 = restric;
  if (u2 >= restric)
    u2 = restric;
  if (u3 >= restric)
    u3 = restric;
  
  if (u1 <=-restric)
    u1 = -restric;
  if (u2 <=-restric)
    u2 = -restric;
  if (u3 <=-restric)
    u3 = -restric;
  
  
  
 //u1 = y_[0]; u2 = y_[1]; u3 = y_[2];
 abaru[0] = yinput_[0];
 abaru[1] = yinput_[1];
 abaru[2] = yinput_[2];
 abaru[3] = yinput_[3];
 abaru[4] = yinput_[4];
 abaru[5] = yinput_[5];
 
 //double kkkkk;
   //kkkkk= cos(thr_th);
// ROS_INFO("kkkkk is %f", kkkkk);
 
  w1_follow = (w1+w4) * cos(thr_th) *Rwr/(2*Rw) + (u2-u1)/Rw + (w1+w3-w2-w4) * Rwr /(4*Rw) * (l1+l2)/(l3+l4) + u3*(l1+l2)/Rw;
  w2_follow = (w2+w3) * cos(thr_th) *Rwr/(2*Rw) - (u2+u1)/Rw + (w2+w4-w1-w3) * Rwr /(4*Rw) * (l1+l2)/(l3+l4) - u3*(l1+l2)/Rw;
  w3_follow = (w2+w3) * cos(thr_th) *Rwr/(2*Rw) - (u2+u1)/Rw + (w1+w3-w2-w4) * Rwr /(4*Rw) * (l1+l2)/(l3+l4) + u3*(l1+l2)/Rw;
  w4_follow = (w1+w4) * cos(thr_th) *Rwr/(2*Rw) + (u2-u1)/Rw + (w4+w2-w1-w3) * Rwr /(4*Rw) * (l1+l2)/(l3+l4) - u3*(l1+l2)/Rw;
  
  //ROS_INFO("w1_follow is %f", w1_follow);
  //ROS_INFO("w2_follow is %f", w2_follow);
  //ROS_INFO("w3_follow is %f", w3_follow);
  //ROS_INFO("w4_follow is %f", w4_follow);
  
  ccc = w1_follow + w2_follow + w3_follow + w4_follow;
  ddd = w1_follow - w2_follow - w3_follow + w4_follow;
  ROS_INFO("callback once OK");
  motor_to_speed_BIG(w1_follow, w2_follow, w3_follow,poseth_follow, w4_follow,msgkkk);
  w_follow = msgkkk.angular.z;
  double temp_w_follow;
  temp_w_follow  = angularw - u3;
  //w_follow = angularw - u3;
  
  ROS_INFO("w_follow is %f", w_follow);
  ROS_INFO("temp_w_follow is %f", temp_w_follow);
  //加入那个算法
  //w_follow = msgkkk.angular.z;

  
  // geometry_msgs::Twist msgojbk;
   msgojbk.linear.x = -msgkkk.linear.x;
     msgojbk.linear.y = msgkkk.linear.y;
       msgojbk.angular.z = msgkkk.angular.z;
   
  car2_pub.publish(msgojbk);
  
   path_pub.publish(path2);
       //msgkkk.linear.x = poseandspeed->twist.twist.linear.x;
       //msgkkk.linear.y = poseandspeed->twist.twist.linear.y;
       //msgkkk.angular.z = poseandspeed->twist.twist.angular.z;
  //car2_pub.publish(msgkkk);
  

}


int main(int argc, char ** argv)
{
  
  posex_follow_path = posey_follow_path = poseth_follow_path = 0;
  theta = 0;
  w_follow = vx_follow = vy_follow = 0;
  w1_follow = w2_follow = w3_follow = w4_follow = 0;
  posex_follow = posey_follow = poseth_follow = 0;
  u1 = u2 = u3 = 0;
  w1 = w2 = w3 = w4 = 0;
  aaa = 0;
  bbb = 0;
  ccc = 0;
  ddd = 0;
  m_ = 30;   // 6*N+6*Nu
  period_ = 0.1;
  gamma_ = 0.005;
  

   Q_.resize(9, 9);                                         // 9x9 
  Q_<< 400 * MatrixXd::Identity(9, 9);
  R_.resize(6, 6);
  R_ << 200 * MatrixXd::Identity(6, 6);                     // 6x6
  tildrI_ << 1, 0, 0, 0, 0, 0,                              // 6x6
             0, 1, 0, 0, 0, 0, 
             1, 0, 1, 0, 0, 0, 
             0, 1, 0, 1, 0, 0, 
             1, 0, 1, 0, 1, 0, 
             0, 1, 0, 1, 0, 1;
  
  abarUmin_ = VectorXd::Ones(6);                            // 6x1
  abarUmin_ *= -0.6;
  abarUmax_ = VectorXd::Ones(6);                            // 6x1
  abarUmax_ *= 0.6;
  abarxmin_ = VectorXd::Ones(9);                            // 9x1
  abarxmin_ *= -10;
  abarxmax_ = VectorXd::Ones(9);                            // 9x1
  abarxmax_ *= 10;
  detabarUmin_ = VectorXd::Ones(6);                         // 6x1
  detabarUmin_ *=  -0.2;
  detabarUmax_ = VectorXd::Ones(6);                         // 6x1
  detabarUmax_ *=  0.2;  
  gxx_ << period_ * MatrixXd::Identity(3, 3);               // 3x3
  z_ << MatrixXd::Zero(3, 3);                               // 3x3
    Matrix<double, 3, 6> G1, G2;
    G1 << gxx_,z_;
    G2 << gxx_, gxx_;
  G_ << G1, G2, G2;                                         // 9x6

  W_ = 2 * G_.transpose() * Q_ * G_;                        // 6x6
  E_.resize(30, 6);
    MatrixXd E1(12,6);
    E1<<-tildrI_, tildrI_;
    MatrixXd E2(18,6);
    E2<<-G_,G_; 
    E_ << E1,E2;                                            // 30x6
  M_.resize(36, 36);                                        // 36x36
    MatrixXd M1_(6, 36);
    MatrixXd M2_(30, 36);
    M1_ << W_, -(E_.transpose());
    M2_ << E_, MatrixXd::Zero(30, 30);
    M_ << M1_, M2_;

  ImH_.resize(36, 36);
  IpHt_.resize(36, 36);
  p_.resize(36);
  Pinfty_.resize(30);
  Minfty_.resize(30);
  yinput_.resize(36); 
  ImH_ << MatrixXd::Identity(36, 36) - M_;                  // 36x36
  IpHt_ << MatrixXd::Identity(36, 36) + M_.transpose();     // 36x36
  Pinfty_ << 1000000000 * VectorXd::Ones(30, 1);                // 30x1
  Minfty_ << - 1000000000 * VectorXd::Ones(30, 1);              // 30x1
  yinput_ <<  VectorXd::Zero(36);  
  abaru << VectorXd::Zero(6);  
  D <<0.25, -0.25, 0;                                          // 3x1
  Dx<< D, D, D;     
  roll = pitch = yaw = 0;
  roll2 = pitch = yaw2 = 0;
  xr_x = yr_y = thr_th =0;
  theta_3x4follow_ << 1, 1, 1, 1, -1, 1, 1, -1, -1/(l1+l2), 1/(l1+l2), -1/(l1+l2), 1/(l1+l2);
  theta_3x4forward_ << 1, 1, 1, 1, -1, 1, 1, -1, -1/(l3+l4),1/(l3+l4), -1/(l3+l4), 1/(l3+l4);
  

  ros::init(argc, argv, "two_car_follow");
  Follow follow;
  
  ros::Time current_time, last_time;
  current_time = ros::Time::now();              //init times
  last_time = ros::Time::now();
  ros::Time fortyfive_seconds = ros::Time::now() + ros::Duration(45);  //45s
  ros::Time ninty_seconds = ros::Time::now() + ros::Duration(90);
  path2.header.stamp=current_time;
  path2.header.frame_id="odom";
  
  
  ros::Rate loop_rate(15);
  ros::MultiThreadedSpinner s(2);  //多线程
  //ros::spin(s); 
  
 while(ros::ok()&& (ros::Time::now()<=fortyfive_seconds) ) {
  
  /*
  w1 = speedddd * 4;
  w2 = speedddd * 6;
  w3 = speedddd * 4.8;
  w4 = speedddd * 5.2;
  */
  w1 = speedddd * 4;
  w2 = speedddd * 4;
  w3 = speedddd * 4;
  w4 = speedddd * 4;
  
   current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x =  (msgojbk.linear.x * cos(poseth_follow) - msgojbk.linear.y * sin(poseth_follow)) * dt;
    double delta_y =  (msgojbk.linear.x * sin(poseth_follow) + msgojbk.linear.y * cos(poseth_follow)) * dt;
    double delta_th =  msgojbk.angular.z * dt;
    posex_follow += delta_x;
    posey_follow += delta_y;
    poseth_follow += delta_th;
    
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = posex_follow;
        this_pose_stamped.pose.position.y = posey_follow;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseth_follow);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odomcar2";
        path2.poses.push_back(this_pose_stamped);
        
    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  
   while(ros::ok() && (ros::Time::now()>fortyfive_seconds && ros::Time::now()<=ninty_seconds )) {
     
  w1 = speedddd * 6;
  w2 = speedddd * 4;
  w3 = speedddd * 5.2;
  w4 = speedddd * 4.8;
/*
  w1 = speedddd * 4;
  w2 = speedddd * 4;
  w3 = speedddd * 4;
  w4 = speedddd * 4;
  */
  current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x =  (msgojbk.linear.x * cos(poseth_follow) - msgojbk.linear.y * sin(poseth_follow)) * dt;
    double delta_y =  (msgojbk.linear.x * sin(poseth_follow) + msgojbk.linear.y * cos(poseth_follow)) * dt;
    double delta_th =  msgojbk.angular.z * dt;
    posex_follow += delta_x;
    posey_follow += delta_y;
    poseth_follow += delta_th;
    
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = posex_follow;
        this_pose_stamped.pose.position.y = posey_follow;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(poseth_follow);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path2.poses.push_back(this_pose_stamped);
        
    last_time = current_time;
  
  
    ros::spinOnce();
    loop_rate.sleep();
    
  }
  

}

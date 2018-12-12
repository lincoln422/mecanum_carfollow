#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#define moveSpeed 100

class transfer
{ public:
     transfer();
  private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    int motor1speed,motor2speed,motor3speed,motor4speed;
    void velMessageReceived(const geometry_msgs::Twist& msg);
};

transfer::transfer()
{ motor1speed=motor2speed=motor3speed=motor4speed=0;
  pub=nh.advertise<geometry_msgs::Twist>("vel2",50);
  sub=nh.subscribe("vel",10,&transfer::velMessageReceived,this);
}

void transfer::velMessageReceived(const geometry_msgs::Twist& msg)
{
  motor1speed=-moveSpeed*(msg.linear.y-msg.linear.x-0.42*msg.angular.z);
  motor2speed=-moveSpeed*(msg.linear.y+msg.linear.x+0.42*msg.angular.z);
  motor3speed=moveSpeed*(msg.linear.y-msg.linear.x+0.42*msg.angular.z);
  motor4speed=moveSpeed*(msg.linear.y+msg.linear.x-0.42*msg.angular.z);
  geometry_msgs::Twist speed;
  speed.linear.x=motor1speed;
  speed.linear.y=motor2speed;
  speed.linear.z=motor3speed;
  speed.angular.z=motor4speed;
    pub.publish(speed); 
}


int main(int argc, char ** argv)
{ ros::init(argc, argv, "vel_transfer");
  transfer tran;
  ros::spin();
}

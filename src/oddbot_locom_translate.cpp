#include "locom_diff/oddbot_locom_translate.h"

oddbot_locom_translate::oddbot_locom_translate(){
	//subscribe to oddbot_boot
	//publish to oddbot_boot_stop
  ros::NodeHandle nodeHandle;
  vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel_translated", 1000);
  vel_sub = nodeHandle.subscribe("cmd_vel", 1000, &oddbot_locom_translate::get_vel, this);  
}
// Callback message
void oddbot_locom_translate::get_vel(const geometry_msgs::Twist::ConstPtr& vel_msg){	
	geometry_msgs::Twist translated_vel;
	translated_vel.linear.x = 128+(128*vel_msg->linear.x);
	translated_vel.angular.z = 128+(128*vel_msg->angular.z);
        if (translated_vel.linear.x > 255)
	{
		translated_vel.linear.x = 255;	
	}
	if (translated_vel.angular.z > 255)
	{
		translated_vel.angular.z = 255;	
	}
	oddbot_locom_translate::vel_pub.publish(translated_vel);
}


int main(int argc, char** argv){
	
	//stop when stop running flag is true
	ros::init(argc, argv, "oddbot_loc_translate");

  oddbot_locom_translate lt = oddbot_locom_translate();
  
  ROS_INFO("oddbot locom translate node started!");  

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}

/*
 * \locom_boot_diff.h
 *
 * \author Chris Dunkers, CMU - cmdunkers@cmu.edu
 * \date October 13, 2014
 */

#ifndef LOCOM_BOOT_DIFF_H_
#define LOCOM_BOOT_DIFF_H_

#include "ros/ros.h"
#include "oddbot_msgs/OddbotBoot.h"
#include "oddbot_msgs/OddbotBootStop.h"
#include "oddbot_lib/get_subnet.h"


class locom_boot_diff{
	public:
		locom_boot_diff();
		bool get_subnet_msg_stop();
		void send_info();
	private:
		bool subnet_msg_stop;
		oddbot_msgs::OddbotBoot module_info_msg;
		ros::Publisher boot_pub;
		ros::Subscriber stop_sub;
		void get_info(const oddbot_msgs::OddbotBootStop::ConstPtr& stop_msg);
		int subnet;
};

int main(int argc, char** argv);

#endif


#ifndef PROBE_H
#define PROBE_H

#include "teensy.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "geometry_msgs/PointStamped.h"

class Probe : public Teensy
{
public:

	int mode 				= 1; // idle
	bool initialized	 	= false;

	Probe();

	void statusClbk(const std_msgs::Int16MultiArray& msg);

	bool sendCmd(int cmd);

private:

	int desired_mode 		= 1;

	ros::NodeHandle n;

	// messages
	std_msgs::Int16 probe_send_msg;

	// publishers
	ros::Publisher probe_cmd_pub;

	// subscribers
	ros::Subscriber probe_status_sub;
	ros::Subscriber probe_contact_sub;
};

#endif 
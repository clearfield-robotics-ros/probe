
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

class Probe
{
public:

	int mode 				= 1; // idle
	int desired_mode 		= 1;
	bool initialized	 	= false;
	bool handshake 			= false;
	bool command_arrived 	= true;

	ros::NodeHandle n;

	// messages
	std_msgs::Int16 probe_send_msg;

	// publishers
	ros::Publisher probe_cmd_pub;

	// subscribers
	ros::Subscriber probe_status_sub;
	ros::Subscriber probe_contact_sub;

	Probe()
	{
		probe_cmd_pub = n.advertise<std_msgs::Int16>("probe_t/probe_cmd_send", 1000);
		probe_status_sub = n.subscribe("probe_t/probe_status_reply", 1000, &Probe::probeStatusClbk, this);
	};
	
	void probeStatusClbk(const std_msgs::Int16MultiArray& msg){
		mode = msg.data.at(0); // first entry is the reported state
		if (mode == desired_mode) handshake = true;
		initialized = (bool)msg.data.at(1);
	}

	bool sendProbeCmd(int cmd) {
		// ROS_INFO("cmd_%d sent to probes", cmd);
		desired_mode = cmd;
		handshake = false;
		command_arrived = false;
		probe_send_msg.data = cmd; // change it to probe mode

		ROS_INFO("Publish Probe Cmd");

		probe_cmd_pub.publish(probe_send_msg); // send the message
	}

	bool notWaiting()
	{
		if (handshake) command_arrived = true;

		if (command_arrived) return true;
		else return false;
	}

	private:
};

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

class Gantry
{
public:

	const float probe_calibration_position = 0.05f; // i.e calibrate probes at 50cm

	int  mode 				= 0; // idle
	bool initialized 		= false;
	bool handshake 			= false;
	bool command_arrived 	= true;

	float pos_cmd 			= 0;
	bool pos_cmd_reached 	= false;
	float carriage_pos;

	ros::NodeHandle n;

	// messages
	std_msgs::Int16MultiArray gantry_send_msg;
	std_msgs::Int16 gantry_cmd_hack_msg;

	// publishers
	ros::Publisher gantry_cmd_pub;
	ros::Publisher gantry_cmd_hack_pub;

	// subscribers
	ros::Subscriber gantry_status_sub;

	// transforms
	tf::TransformBroadcaster br;
	tf::Transform gantry;
	tf::Transform gantry_carriage;

	Gantry()
	{
		gantry_cmd_pub = n.advertise<std_msgs::Int16MultiArray>("gantry/gantry_cmd_send", 1000);
		gantry_cmd_hack_pub = n.advertise<std_msgs::Int16>("gantry/gantry_cmd_hack_send", 1000);
		gantry_status_sub = n.subscribe("gantry/gantryStat", 1000, &Gantry::gantryStatusClbk, this);
	};

	void gantryStatusClbk(const std_msgs::Int16MultiArray& msg) {
		// ROS_INFO("gantry");
		mode = msg.data.at(0); // first entry is the reported state
		initialized = (bool)msg.data.at(1);

		pos_cmd_reached = (bool)msg.data.at(2); // third entry is the status of whether or not the command position has been reached
		if (pos_cmd_reached == 0) handshake = true;

		// ROS_INFO("gantry_mode=%d, gantry_initialized=%d", gantry_mode, gantry_initialized);
	    carriage_pos = (float)msg.data.at(3)/1000.0; // fourth entry is the gantry carriage position [mm->m]
	    
	    gantry_carriage.setOrigin( tf::Vector3(0.09+carriage_pos,-0.13,0.365)); // update origin of gantry carriage coordinate frame
		gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform}
	}

	void sendGantryPosCmd(){

		handshake = false;
		command_arrived = false;

		gantry_send_msg.data.clear();
		gantry_send_msg.data.push_back(1); // safe to move
		gantry_send_msg.data.push_back((int)(pos_cmd*1000)); // input the position [mm]

		gantry_cmd_pub.publish(gantry_send_msg); // send the message
	}

	void sendGantryIdleCmd(){
		gantry_send_msg.data.clear(); // flush out previous data
		gantry_send_msg.data.push_back(0); // not safe to move
		gantry_send_msg.data.push_back(0); // input the position [mm]
		gantry_cmd_pub.publish(gantry_send_msg); // send the message
	}

	void updateGantryState() {
		gantry_cmd_hack_msg.data = 3;
		gantry_cmd_hack_pub.publish(gantry_cmd_hack_msg);
	}

	bool notWaiting()
	{
		if (handshake) command_arrived = true;

		if (command_arrived) return true;
		else return false;
	}

private:
};
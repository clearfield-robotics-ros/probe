
#include "teensy.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "geometry_msgs/PointStamped.h"

class Gantry : public Teensy
{
public:

	bool initialized 		= false;
	bool pos_cmd_reached 	= false;
	float carriage_pos;

	Gantry();

	void statusClbk(const std_msgs::Int16MultiArray& msg);

	void sendPosCmd(float _pos_cmd);

	void sendIdleCmd();

	void updateState();

private:

	int  mode 		= 0; // idle
	float pos_cmd	= 0;

	int calibration_position;

	ros::NodeHandle n;

	// messages
	std_msgs::Int16MultiArray gantry_send_msg;
	std_msgs::Int16 gantry_mode_msg;

	// publishers
	ros::Publisher gantry_cmd_pub;
	ros::Publisher gantry_mode_pub;

	// subscribers
	ros::Subscriber gantry_status_sub;
};
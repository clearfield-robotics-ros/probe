
#ifndef CLASSIFY_H
#define CLASSIFY_H

#include "circle_type.h"
#include "circle_fit.h"
#include "ros/ros.h"
#include "probe/mine.h"
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
#include "visualization_msgs/Marker.h"

class Classify : public Circle_Type
{
public:

	Classify();

private:

	Circle_Fit circle_fit;

	struct Contact {
		geometry_msgs::PointStamped point;
		bool type;
	};

	int landmineCount = 0;
	std::vector<Contact> contact;

	// locations of gantry and probe carriages
	float probe_carriage_pos, gantry_carriage_pos;
	bool solid_contact 		= false;

	ros::NodeHandle n;

	// subscribers
	ros::Subscriber new_mine_data;
	ros::Subscriber gantry_status_sub;
	ros::Subscriber probe_status_sub;
	ros::Subscriber probe_contact_sub;

	// messages
	probe::mine mine_estimate;

	// publishers
	ros::Publisher mine_estimate_pub;
	ros::Publisher probe_contact_pub;
	ros::Publisher vis_pub;

	// transforms
	tf::TransformBroadcaster br;
	tf::Transform gantry;
	tf::Transform gantry_carriage;
	tf::TransformListener probe_listener;
	tf::Transform probe_rail;
	tf::Transform probe_tip;

	void newMineClbk(const probe::mine& msg);

	void gantryStatusClbk(const std_msgs::Int16MultiArray& msg);

	void probeStatusClbk(const std_msgs::Int16MultiArray& msg);

	void probeContactClbk(const std_msgs::Int16MultiArray& msg);

	void calulateResults();

	void printResults();

	void viz_mine();

	void viz_results();

	void viz_text(std::string label, float x, float y, float z);

	int viz_prove_index = 0;
	void viz_probe(int i);
};

#endif
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
	Probe()
	{


	// publishers
	probe_cmd_pub = n.advertise<std_msgs::Int16>("probe_cmd_send", 1000);
	probe_contact_pub = n.advertise<geometry_msgs::PointStamped>("probe_contact_send", 1000);
	gantry_cmd_pub = n.advertise<std_msgs::Int16MultiArray>("gantry_cmd_send", 1000);

	// subscribers
	probe_status_sub = n.subscribe("probe_status_reply", 1000, &Probe::probeStatusClbk, this);
	probe_contact_sub = n.subscribe("probe_contact_reply", 1000, &Probe::probeContactClbk, this);
	gantry_status_sub = n.subscribe("gantryStat", 1000, &Probe::gantryStatusClbk, this);
	};
	
	ros::NodeHandle n;

	// Callback functions

void probeStatusClbk(const std_msgs::Int16MultiArray& msg){
	probe_mode = msg.data.at(0); // first entry is the reported state
	probe_carriage_pos = calculateProbeExtension((float)msg.data.at(2)); // second entry is the probe carriage position [mm]
	ROS_INFO("%f", probe_carriage_pos);
	probe_tip.setOrigin( tf::Vector3(0,0.4+probe_carriage_pos,0)); // update origin of probe tip coordinate frame [m]
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
	probes_initialized = (bool)msg.data.at(1);
	probe_cycle_complete = (bool)msg.data.at(2);
}

void probeContactClbk(const std_msgs::Int16MultiArray& msg){
	probe_carriage_pos = calculateProbeExtension((float)msg.data.at(3)); // second entry is the probe carriage position [mm]
	probe_tip.setOrigin( tf::Vector3(0,0.4+probe_carriage_pos,0)); // update origin of probe tip coordinate frame
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
	tf::StampedTransform probe_tf;
	probe_listener.lookupTransform("probe_tip","base_link",ros::Time(0),probe_tf);
	geometry_msgs::PointStamped cp; // single contact point (cp)
	tf::Vector3 probe_tip_origin;
	probe_tip_origin = probe_tf.getOrigin();
	cp.point.x = probe_tip_origin.x();
	cp.point.y = probe_tip_origin.y();
	cp.point.z = probe_tip_origin.z();
	probe_contact_pub.publish(cp); // publish to save in rosbag
	contact_points.push_back(cp); // save into vector for internal processing
}

void gantryStatusClbk(const std_msgs::Int16MultiArray& msg){
	gantry_mode = msg.data.at(0); // first entry is the reported state
	gantry_initialized = (bool)msg.data.at(1);
	gantry_pos_cmd_reached = (bool)msg.data.at(2); // third entry is the status of whether or not the command position has been reached
    gantry_carriage_pos = (float)msg.data.at(3)/1000.0; // second entry is the gantry carriage position [mm->m]
    gantry_carriage.setOrigin( tf::Vector3(0,0.1+gantry_carriage_pos,0)); // update origin of gantry carriage coordinate frame
    br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform}
}

struct point2D { float x, y; };

struct circle {
    point2D center;
    float rad;
};

// messages
std_msgs::Int16 probe_send_msg;
std_msgs::Int16MultiArray gantry_send_msg;

// publishers
ros::Publisher probe_contact_pub;
ros::Publisher probe_cmd_pub;
ros::Publisher gantry_cmd_pub;

// subscribers
ros::Subscriber probe_status_sub;
ros::Subscriber probe_contact_sub;
ros::Subscriber gantry_status_sub;

// transforms 
tf::TransformBroadcaster br;
tf::TransformListener probe_listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

std::vector<geometry_msgs::PointStamped> contact_points;

// Helper functions

float calculateProbeExtension(float encoder_counts);

std::vector<float> generateSamplingPoints(float center);

// Shape classification functions

float calcRadius(point2D& cc, std::vector<point2D>& points);

point2D circumcenter(const std::vector<point2D>& points);

circle calcCircle(std::vector<point2D>& points);

circle classify(const std::vector<geometry_msgs::PointStamped>& pts3d);

// Initialization functions

void initializeGantry();

void initializeProbes();

// Command functions

void sendGantryPosCmd(float pos);

void sendGantryIdleCmd();

void sendProbeInsertCmd();

void printResults();

// probing parameters
std::vector<double> target_x = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7}; // [m]
std::vector<double> target_y = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
std::vector<bool> isMine =     {1, 1, 1, 0, 0, 0}; // 1 if mine, 0 if non-mine
int num_probes_per_obj = 5;
float spacing_between_probes = 0.02; // [m] = 2cm
float sample_width = num_probes_per_obj*spacing_between_probes;
float max_radius = 0.15; // [m] = 15cm

int num_targets = target_x.size();
int current_target_id = 0;
int sampling_point_index = 0;
bool sampling_points_generated = false;
bool indiv_inspection_complete = false;
bool full_inspection_complete = false;

// locations of gantry and probe carriages
float probe_carriage_pos;
float gantry_carriage_pos;

// standard commands
int probe_idle_cmd =            1;
int probe_insert_cmd =          2;
int probe_initialize_cmd =      3;

// initialize modes
int probe_mode =                1; // idle
int gantry_mode =               0; // idle

// initialization flags
bool both_initialized =         false; // both probes and gantry

bool gantry_initialized =       false;
bool gantry_init_cmd_sent =     false;
bool gantry_pos_cmd_sent =      false;
bool gantry_pos_cmd_reached =   false;

bool probes_initialized =       false;
bool probe_init_cmd_sent =      false;
bool probe_insert_cmd_sent =    false;
bool probe_cycle_complete =     false;
bool demo_complete =            false;

private:

};

// setup in arduino
  // arr.layout.dim = (std_msgs::MultiArrayDimension *)
  // malloc(sizeof(std_msgs::MultiArrayDimension)*2);
  // arr.layout.dim[0].label = "height";
  // arr.layout.dim[0].size = 4;
  // arr.layout.dim[0].stride = 1;
  // arr.layout.data_offset = 0;
  // arr.data = (int *)malloc(sizeof(int)*8);
  // arr.data_length = 4;
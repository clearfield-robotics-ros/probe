#include <probe/probe.h>

#define M_PI 3.14159265358979323846

int probe_mode = 1; // Idle
int gantry_mode = 0;

int probe_cmd = 1; // Idle
int gantry_cmd = 0;

std_msgs::Int32 probe_send_msg;
std_msgs::Int32 gantry_send_msg;
// std_msgs::Float32 gantry_pos_cmd_msg;

static tf::TransformBroadcaster br;
tf::TransformListener probe_listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

double start_time = ros::Time::now().toSec();
double delay_before_start = 5;
bool start_routine = false;

std::vector<double> target_centers = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
int num_targets = target_centers.size();
int current_target_id = 0;
bool target_probing_complete = false;

float probe_carriage_pos;
float gantry_carriage_pos;

int gantry_initialize_cmd[] = {1,0};
int gantry_idle_cmd[] = {0,0};

int gantry_initialize_cmd_received[] = {1,0,-1};
int gantry_initialization_complete[] = {1,1,-1};
int gantry_idle_cmd_received[] = 	   {0,1,-1};
int gantry_pos_cmd_received[] =        {3,1,0};
int gantry_pos_cmd_reached[] =         {3,1,1};


int probe_idle_cmd = 1;
int probe_insert_cmd = 2;
int probe_initialize_cmd = 3;

int probe_initialize_cmd_received[] = {3,0,-1};
int probe_initialization_complete[] = {3,1,-1};
int probe_idle_cmd_received[] = 	  {1,1,-1};
int probe_insert_cmd_received[] =     {2,1,0};
int probe_insertion_complete[] =      {2,1,1};


// int gantry_init_complete;
// int probe_init_complete;
std::vector<geometry_msgs::PointStamped> contact_points;




float calculateProbeExtension(float encoder_counts){
	float probe_encoder_count_per_rev = 12;
	float probe_gear_ratio = 27;
	float probe_lead = 0.008; // [m]
	return encoder_counts*probe_lead/(probe_gear_ratio*probe_encoder_count_per_rev);
}

void probeStatusClbk(const std_msgs::Int32MultiArray& msg){
	probe_mode = msg.data[0]; // first entry is the reported state
	probe_carriage_pos = calculateProbeExtension((float)msg.data[1]); // second entry is the probe carriage position [mm]
	probe_tip.setOrigin( tf::Vector3(0,0.4+probe_carriage_pos,0)); // update origin of probe tip coordinate frame
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
}

void probeContactClbk(const std_msgs::Int32MultiArray& msg){
	probe_carriage_pos = calculateProbeExtension((float)msg.data[1]); // second entry is the probe carriage position [mm]
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
	contact_points.push_back(cp);
}

void gantryStatusClbk(const std_msgs::Int32MultiArray& msg){
	gantry_mode = msg.data[0]; // first entry is the reported state
	gantry_carriage_pos = (float)msg.data[1]/1000.0; // second entry is the gantry carriage position [mm->m]
	// gantry_pos_cmd_reached = msg.data[2]; // third entry is the status of whether or not the command position has been reached
	gantry_init_complete = msg.data[3]; // fourth entry is whether or not the initialization is complete
	gantry_carriage.setOrigin( tf::Vector3(0,0.1+gantry_carriage_pos,0)); // update origin of gantry carriage coordinate frame
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform
}

int main(int argc, char **argv)
{
	ros::NodeHandle n;

	ros::init(argc, argv, "probe");
	ros::Rate loop_rate(10);

	// publishers
	ros::Publisher probe_cmd_pub = n.advertise<std_msgs::Int32>("probe_cmd_send", 1000);
	ros::Publisher gantry_cmd_pub = n.advertise<std_msgs::Int32MultiArray>("gantry_cmd_send", 1000);
	// ros::Publisher gantry_pos_cmd_pub = n.advertise<std_msgs::Float32>("gantry_pos_cmd", 1000);

  	// subscribers
	ros::Subscriber probe_status_sub = n.subscribe("probe_status_reply", 1000, probeStatusClbk);
	ros::Subscriber probe_contact_sub = n.subscribe("probe_contact_reply", 1000, probeContactClbk);
	ros::Subscriber gantry_status_sub = n.subscribe("gantry_status_reply", 1000, gantryStatusClbk);

	// initialize transforms
	gantry.setOrigin( tf::Vector3(0.2,-0.2,0.4));
	gantry.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(gantry,ros::Time::now(), "base_link", "gantry"));

	gantry_carriage.setOrigin( tf::Vector3(0,0.1,0));
	gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage"));

	probe_rail.setOrigin( tf::Vector3(0,-0.1,-0.2));
	tf::Matrix3x3 m_rot;
	m_rot.setEulerYPR(0, -30*M_PI/180, 0);
	// Convert into quaternion
	tf::Quaternion quat;
	m_rot.getRotation(quat);
	probe_rail.setRotation(tf::Quaternion(quat));
	br.sendTransform(tf::StampedTransform(probe_rail,ros::Time::now(), "gantry_carriage", "probe_rail"));

	probe_tip.setOrigin( tf::Vector3(0,0.4,0));
	probe_tip.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip"));

	while (ros::ok())
	{
		if(ros::Time::now().toSec()-delay_before_start>start_time){
			start_routine = true; // include a buffer between booting up system and the start of the routine
		}

		if(start_routine){
			switch(gantry_mode){
				case 0:
				gantry_cmd = 1; // Initialize
				break;
				case 1:
				break;
				case 2: // should not enter this mode during probing test
				break;
				case 3:
				break;

				default:
				break;	
			}

			switch(probe_mode){
				case 0:
				if (gantry_init_complete==0){ // if gantry initialization is not complete
					break; // do nothing
				}
				else if (gantry_init_complete==1){ // if gantry initialization is complete
					probe_cmd = 3; // Calibrate
				}
				break;
				case 1:
				if (probe_init_complete==0){ // if probe initialization is not complete
					break; // do nothing
				}
				else if (probe_init_complete==1){ // if gantry initialization is complete
					probe_cmd = 0; // Return to home and wait for gantry to move to first probing position
				}
				break;
				case 2:
				break;
				case 3:
				if(current_target_id<num_targets){
				// gantry_cmd = GantryCmd::Position;
				} else {
				// gantry_cmd = GantryCmd::Home;
				}
				current_target_id++;
				break;
				default:
				break;	
			}

			switch(probe_cmd){
				case 0:
			// probe_cmd_send.data = "Home";
			// probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
				break;
				case 1:
			// probe_mode_cmd_msg.data = "Initialize";
			// probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
				break;
				case 2:
			// probe_mode_cmd_msg.data = "Probe";
			// probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
				break;
				case 3:
			// probe_mode_cmd_msg.data = "Stop";
			// probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
				break;
				default:
				break;					
			}

			switch(gantry_cmd){
				case 0:
			// gantry_mode_cmd_msg.data = "Home";
			// gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
				break;
				case 1:
			// gantry_mode_cmd_msg.data = "Initialize";
			// gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
				break;
				case 2:
			// gantry_mode_cmd_msg.data = "Position";
			// gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
			// gantry_pos_cmd_msg.data = target_centers[current_target_id];
			// gantry_pos_cmd_pub.publish(gantry_pos_cmd_msg);
				break;
				case 3:
			// gantry_mode_cmd_msg.data = "Stop";
			// gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
				break;
				default:
				break;	
			}
		}


		ROS_INFO("Probe Cmd: %d --- Probe Mode: %d ------ Gantry Cmd: %d --- Gantry Mode: %d",probe_cmd,probe_mode,gantry_cmd,gantry_mode);
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_8543130373635161B1C0-if00
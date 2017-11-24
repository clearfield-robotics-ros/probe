#include <probe/probe.h>

#define M_PI 3.14159265358979323846

int probe_mode = 0;
int gantry_mode = 0;

std_msgs::Int32 probe_send_msg;
std_msgs::Int32 gantry_send_msg;
// std_msgs::Float32 gantry_pos_cmd_msg;

static tf::TransformBroadcaster br;
tf::TransformListener probe_listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

std::vector<double> target_centers = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
int num_targets = target_centers.size();
int current_target_id = 0;

float probe_carriage_pos;
float gantry_carriage_pos;

int gantry_pos_cmd_reached;
std::vector<geometry_msgs::PointStamped> contact_points;


void probeStatusClbk(const std_msgs::Int32MultiArray& msg){
	probe_mode = msg.data[0]; // first entry is the reported state
	probe_carriage_pos = (float)msg.data[1]/1000.0; // second entry is the probe carriage position [mm]
	probe_tip.setOrigin( tf::Vector3(0,0.4+probe_carriage_pos,0)); // update origin of probe tip coordinate frame
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
}

void probeContactClbk(const std_msgs::Int32MultiArray& msg){
	probe_carriage_pos = (float)msg.data[1]/1000.0; // second entry is the probe carriage position [mm]
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
	gantry_carriage_pos = (float)msg.data[1]/1000.0; // second entry is the gantry carriage position [mm]
	gantry_pos_cmd_reached = msg.data[2];
	gantry_carriage.setOrigin( tf::Vector3(0,0.1+gantry_carriage_pos,0)); // update origin of gantry carriage coordinate frame
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform
}

int main(int argc, char **argv)
{
	ros::NodeHandle n;

	ros::init(argc, argv, "probe");
	ros::Rate loop_rate(10);

	// n.getParam("target_centers", target_centers);

// publishers
	ros::Publisher probe_send_pub = n.advertise<std_msgs::Int32>("probe_cmd_send", 1000);
	ros::Publisher gantry_send_pub = n.advertise<std_msgs::Int32MultiArray>("gantry_cmd_send", 1000);
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

		switch(gantry_mode){
			case 0:
			break;
			case 1:
			// probe_cmd = ProbeCmd::Initialize;
			break;
			case 2:
			break;
			case 3:
			// probe_cmd = ProbeCmd::Probe; // issue probe command
			break;

			default:
			break;	
		}

		switch(probe_mode){
			case 0:
			break;
			case 1:
			// gantry_cmd = GantryCmd::Position; // move to first position
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

		// switch(probe_cmd){
		// 	case ProbeCmd::Home:
		// 	probe_mode_cmd_msg.data = "Home";
		// 	probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
		// 	break;
		// 	case ProbeCmd::Initialize:
		// 	probe_mode_cmd_msg.data = "Initialize";
		// 	probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
		// 	break;
		// 	case ProbeCmd::Probe:
		// 	probe_mode_cmd_msg.data = "Probe";
		// 	probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
		// 	break;
		// 	case ProbeCmd::Stop:
		// 	probe_mode_cmd_msg.data = "Stop";
		// 	probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
		// 	break;
		// 	default:
		// 	break;					
		// }

		// switch(gantry_cmd){
		// 	case GantryCmd::Home:
		// 	gantry_mode_cmd_msg.data = "Home";
		// 	gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
		// 	break;
		// 	case GantryCmd::Initialize:
		// 	gantry_mode_cmd_msg.data = "Initialize";
		// 	gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
		// 	break;
		// 	case GantryCmd::Position:
		// 	gantry_mode_cmd_msg.data = "Position";
		// 	gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
		// 	gantry_pos_cmd_msg.data = target_centers[current_target_id];
		// 	gantry_pos_cmd_pub.publish(gantry_pos_cmd_msg);
		// 	break;
		// 	case GantryCmd::Stop:
		// 	gantry_mode_cmd_msg.data = "Stop";
		// 	gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
		// 	break;
		// 	default:
		// 	break;	
		// }


		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_8543130373635161B1C0-if00
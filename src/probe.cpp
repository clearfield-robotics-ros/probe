#include <probe/probe.h>

#define M_PI 3.14159265358979323846

enum class ProbeMode{Home, Initialized, Probing, DoneProbing, Stop}; // enum class prevents name conflicts
enum class GantryMode{Home, Initialized, Positioning, DonePositioning, Stop};

enum class ProbeCmd{Home, Initialize, Probe, Stop};
enum class GantryCmd{Home, Initialize, Position, Stop};

ProbeMode probe_mode = ProbeMode::Home;
GantryMode gantry_mode = GantryMode::Home;

ProbeCmd probe_cmd = ProbeCmd::Home;
GantryCmd gantry_cmd = GantryCmd::Home;

ProbeMode convertProbeMode(const std::string& str);
GantryMode convertGantryMode(const std::string& str);

std_msgs::String probe_mode_cmd_msg;
std_msgs::String gantry_mode_cmd_msg;
std_msgs::Float32 gantry_pos_cmd_msg;

static tf::TransformBroadcaster br;
tf::TransformListener probe_listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

std::vector<double> target_centers = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
int num_targets = target_centers.size();
int current_target_id = 0;

void probeModeClbk(const std_msgs::String& msg){
	probe_mode = convertProbeMode(msg.data);
}

void probeForceClbk(const std_msgs::Float32& msg){
	// convert the origin of the probe tip to a new point and save it
}

void probeCarriagePosClbk(const std_msgs::Float32& msg){
	probe_tip.setOrigin( tf::Vector3(0,0.4+msg.data,0));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip"));
	tf::StampedTransform probe_tf;
	probe_listener.lookupTransform("probe_tip","base_link",ros::Time(0),probe_tf);
	geometry_msgs::PointStamped point;
	tf::Vector3 probe_tip_origin;
	probe_tip_origin = probe_tf.getOrigin();
	point.point.x = probe_tip_origin.x();
	point.point.y = probe_tip_origin.y();
	point.point.z = probe_tip_origin.z();
}

void gantryModeClbk(const std_msgs::String& msg){
	gantry_mode = convertGantryMode(msg.data);

}

void gantryCarriagePosClbk(const std_msgs::Float32& msg){
	gantry_carriage.setOrigin( tf::Vector3(0,msg.data,0));
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage"));
}

ProbeMode convertProbeMode(const std::string& str){
	if(str=="Home") return ProbeMode::Home;
	else if (str=="Initialized") return ProbeMode::Initialized;
	else if (str=="Probing") return ProbeMode::Probing;
	else if (str=="DoneProbing") return ProbeMode::DoneProbing;
	else if (str=="Stop") return ProbeMode::Stop;
}

GantryMode convertGantryMode(const std::string& str){
	if(str=="Home") return GantryMode::Home;
	else if (str=="Initialized") return GantryMode::Initialized;
	else if (str=="Positioning") return GantryMode::Positioning;
	else if (str=="DonePositioning") return GantryMode::DonePositioning;
	else if (str=="Stop") return GantryMode::Stop;
}


int main(int argc, char **argv)
{
	ros::NodeHandle n;

	ros::init(argc, argv, "probe");
	ros::Rate loop_rate(10);

	n.getParam("target_centers", target_centers);

// publishers
	ros::Publisher probe_mode_cmd_pub = n.advertise<std_msgs::String>("probe_mode_cmd", 1000);
	ros::Publisher gantry_mode_cmd_pub = n.advertise<std_msgs::String>("gantry_mode_cmd", 1000);
	ros::Publisher gantry_pos_cmd_pub = n.advertise<std_msgs::Float32>("gantry_pos_cmd", 1000);

  // subscribers
	ros::Subscriber probe_mode_sub = n.subscribe("probe_mode", 1000, probeModeClbk);
	ros::Subscriber probe_force_sub = n.subscribe("probe_force", 1000, probeForceClbk);
	ros::Subscriber probe_carriage_pos_sub = n.subscribe("probe_carriage_pos", 1000, probeCarriagePosClbk);
	ros::Subscriber gantry_mode_sub = n.subscribe("gantry_mode", 1000, gantryModeClbk);
	ros::Subscriber gantry_carriage_pos_sub = n.subscribe("gantry_carriage_pos", 1000, gantryCarriagePosClbk);

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
			case GantryMode::Home:
			break;
			case GantryMode::Initialized:
			probe_cmd = ProbeCmd::Initialize;
			break;
			case GantryMode::Positioning:
			break;
			case GantryMode::DonePositioning:
			probe_cmd = ProbeCmd::Probe; // issue probe command
			break;
			case GantryMode::Stop:
			break;
			default:
			break;	
		}

		switch(probe_mode){
			case ProbeMode::Home:
			break;
			case ProbeMode::Initialized:
			gantry_cmd = GantryCmd::Position; // move to first position
			break;
			case ProbeMode::Probing:
			break;
			case ProbeMode::DoneProbing:
			if(current_target_id<num_targets){
				gantry_cmd = GantryCmd::Position;
			} else {
				gantry_cmd = GantryCmd::Home;
			}
			current_target_id++;
			break;
			case ProbeMode::Stop:
			break;
			default:
			break;	
		}

		switch(probe_cmd){
			case ProbeCmd::Home:
			probe_mode_cmd_msg.data = "Home";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeCmd::Initialize:
			probe_mode_cmd_msg.data = "Initialize";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeCmd::Probe:
			probe_mode_cmd_msg.data = "Probe";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeCmd::Stop:
			probe_mode_cmd_msg.data = "Stop";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			default:
			break;					
		}

		switch(gantry_cmd){
			case GantryCmd::Home:
			gantry_mode_cmd_msg.data = "Home";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
			break;
			case GantryCmd::Initialize:
			gantry_mode_cmd_msg.data = "Initialize";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
			break;
			case GantryCmd::Position:
			gantry_mode_cmd_msg.data = "Position";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
			gantry_pos_cmd_msg.data = target_centers[current_target_id];
			gantry_pos_cmd_pub.publish(gantry_pos_cmd_msg);
			break;
			case GantryCmd::Stop:
			gantry_mode_cmd_msg.data = "Stop";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);
			break;
			default:
			break;	
		}


		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_8543130373635161B1C0-if00
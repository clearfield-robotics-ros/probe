#include <probe/probe.h>

#define M_PI 3.14159265358979323846

enum class ProbeMode{Home, Extend, Retract, Stop}; // enum class prevents name conflicts
enum class GantryMode{Home, Sweep, Position, Stop};

enum class ProbeCmd{Home, Initialize, Probe, Stop};
enum class GantryCmd{Home, Initialize, Sweep, Position, Stop};

ProbeMode probe_mode = ProbeMode::Home;
GantryMode gantry_mode = GantryMode::Home;

// ProbeMode convertProbeMode(const std::string& str);
// GantryMode convertGantryMode(const std::string& str);

std_msgs::String probe_mode_cmd_msg;
std_msgs::String gantry_mode_cmd_msg;

static tf::TransformBroadcaster br;
tf::TransformListener listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

std::vector<double> target_centers;

void probeModeClbk(const std_msgs::String& msg){
	std::string received_probe_mode = msg.data;
	// if (receivedProbeMode==probe_mode_cmd_msg.data)
	// {
	// 	probeMode = convertProbeMode(receivedProbeMode);
	// }
}

void probeForceClbk(const std_msgs::Float32& msg){
	// tell probe to retract
	probe_mode = ProbeMode::Idle;
	// convert the origin of the probe tip to a new point and save it

}

void probeCarriagePosClbk(const std_msgs::Float32& msg){
	probe_tip.setOrigin( tf::Vector3(0,0.4+msg.data,0));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip"));
}

void gantryModeClbk(const std_msgs::String& msg){
	std::string received_gantry_mode = msg.data;
	// if (receivedGantryMode==gantry_mode_cmd_msg.data)
	// {
	// 	gantryMode = convertGantryMode(receivedGantryMode);
	// }
}

void gantryCarriagePosClbk(const std_msgs::Float32& msg){
	gantry_carriage.setOrigin( tf::Vector3(0,msg.data,0));
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage"));
}

// ProbeMode convertProbeMode(const std::string& str){
// 	if(str=="Initialize") return ProbeMode::Initialize;
// 	else if (str=="Idle") return ProbeMode::Idle;
// 	else if (str=="Probe") return ProbeMode::Probe;
// 	else if (str=="Stop") return ProbeMode::Stop;
// }

// GantryMode convertGantryMode(const std::string& str){
// 	if(str=="Initialize") return GantryMode::Initialize;
// 	else if (str=="Idle") return GantryMode::Idle;
// 	else if (str=="Position") return GantryMode::Position;
// 	else if (str=="Stop") return GantryMode::Stop;
// }

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
	ros::Subscriber gantry_pos_actual_sub = n.subscribe("gantry_pos_actual", 1000, gantryCarriagePosClbk);

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
		switch(probe_mode){
			case ProbeMode::Initialize:
			probe_mode_cmd_msg.data = "Initialize";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeMode::Idle:
			probe_mode_cmd_msg.data = "Idle";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeMode::Probe:
			probe_mode_cmd_msg.data = "Probe";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;
			case ProbeMode::Stop:
			probe_mode_cmd_msg.data = "Stop";
			probe_mode_cmd_pub.publish(probe_mode_cmd_msg);
			break;				
			default:
			break;					
		}

		switch(gantry_mode){
			case GantryMode::Initialize:
			gantry_mode_cmd_msg.data = "Initialize";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);		
			break;
			case GantryMode::Idle:
			gantry_mode_cmd_msg.data = "Idle";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);		
			break;
			case GantryMode::Position:
			gantry_mode_cmd_msg.data = "Position";
			gantry_mode_cmd_pub.publish(gantry_mode_cmd_msg);		
			break;
			case GantryMode::Stop:
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
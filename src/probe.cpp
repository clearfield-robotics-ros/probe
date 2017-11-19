#include <probe/probe.h>

enum class ProbeMode{Initialize, Idle, Probe, Stop}; // enum class prevents name conflicts
enum class GantryMode{Initialize, Idle, Position, Stop};

ProbeMode probeMode = ProbeMode::Idle;
GantryMode gantryMode = GantryMode::Initialize;

ProbeMode convertProbeMode(const std::string& str);
GantryMode convertGantryMode(const std::string& str);

std_msgs::String probe_mode_cmd_msg;
std_msgs::String gantry_mode_cmd_msg;

static tf::TransformBroadcaster br;

tf::Transform gantry;
tf::Transform gantryCarriage;
tf::Transform probeCarriage;
tf::Transform probeTip;

void probeModeClbk(const std_msgs::String& msg){
	std::string receivedProbeMode = msg.data;
	// if (receivedProbeMode==probe_mode_cmd_msg.data)
	// {
	// 	probeMode = convertProbeMode(receivedProbeMode);
	// }
}

void probeForceClbk(const std_msgs::Float32& msg){}

void probeCarriagePosClbk(const std_msgs::Float32& msg){

}

void gantryModeClbk(const std_msgs::String& msg){
	std::string receivedGantryMode = msg.data;
	// if (receivedGantryMode==gantry_mode_cmd_msg.data)
	// {
	// 	gantryMode = convertGantryMode(receivedGantryMode);
	// }
}

void gantryCarriagePosClbk(const std_msgs::Float32& msg){

}

ProbeMode convertProbeMode(const std::string& str){
	if(str=="Initialize") return ProbeMode::Initialize;
	else if (str=="Idle") return ProbeMode::Idle;
	else if (str=="Probe") return ProbeMode::Probe;
	else if (str=="Stop") return ProbeMode::Stop;
}

GantryMode convertGantryMode(const std::string& str){
	if(str=="Initialize") return GantryMode::Initialize;
	else if (str=="Idle") return GantryMode::Idle;
	else if (str=="Position") return GantryMode::Position;
	else if (str=="Stop") return GantryMode::Stop;
}

int main(int argc, char **argv)
{
	ros::NodeHandle n;

	ros::init(argc, argv, "probe");
	ros::Rate loop_rate(10);

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

	gantryCarriage.setOrigin( tf::Vector3(0.1,0,0));
	gantryCarriage.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(gantryCarriage,ros::Time::now(), "gantry", "gantry_carriage"));

	probeCarriage.setOrigin( tf::Vector3(0,0,0));
	probeCarriage.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probeCarriage,ros::Time::now(), "gantry_carriage", "probe_carriage"));

	probeTip.setOrigin( tf::Vector3(0.4,0,0));
	probeTip.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probeTip,ros::Time::now(), "probe_carriage", "probe_tip"));

	while (ros::ok())
	{
		switch(probeMode){
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

		switch(gantryMode){
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
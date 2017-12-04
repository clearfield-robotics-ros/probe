
#include <probe.h>

Probe::Probe() {
	probe_cmd_pub = n.advertise<std_msgs::Int16>("probe_t/probe_cmd_send", 1000);
	probe_status_sub = n.subscribe("probe_t/probe_status_reply", 1000, &Probe::statusClbk, this);
}

void Probe::statusClbk(const std_msgs::Int16MultiArray& msg){
	mode = msg.data.at(0); // first entry is the reported state
	if (mode == desired_mode) handshake = true;
	initialized = (bool)msg.data.at(1);
}

bool Probe::sendCmd(int cmd) {
	desired_mode = cmd;
	handshake = false;
	command_arrived = false;
	probe_send_msg.data = cmd; // change it to probe mode

	probe_cmd_pub.publish(probe_send_msg); // send the message
}
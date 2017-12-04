
#include <gantry.h>

Gantry::Gantry() {

	gantry_cmd_pub = n.advertise<std_msgs::Int16MultiArray>("gantry/gantry_cmd_send", 1000);
	gantry_mode_pub = n.advertise<std_msgs::Int16>("gantry/gantry_cmd_hack_send", 1000);
	gantry_status_sub = n.subscribe("gantry/gantryStat", 1000, &Gantry::statusClbk, this);
};


void Gantry::statusClbk(const std_msgs::Int16MultiArray& msg) {

	mode = msg.data.at(0); // first entry is the reported state
	initialized = (bool)msg.data.at(1);

	pos_cmd_reached = (bool)msg.data.at(2); // third entry is the status of whether or not the command position has been reached
	if (pos_cmd_reached == 0) handshake = true;

    carriage_pos = (float)msg.data.at(3)/1000.0; // fourth entry is the gantry carriage position [mm->m]
}

void Gantry::sendPosCmd(float _pos_cmd) {

	handshake = false;
	command_arrived = false;

	pos_cmd = _pos_cmd;

	gantry_send_msg.data.clear();
	gantry_send_msg.data.push_back(1); // safe to move
	gantry_send_msg.data.push_back((int)(pos_cmd*1000)); // input the position [mm]

	gantry_cmd_pub.publish(gantry_send_msg); // send the message
}

void Gantry::sendIdleCmd(){

	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data.push_back(0); // not safe to move
	gantry_send_msg.data.push_back(0); // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
}

void Gantry::updateState() {
	
	gantry_mode_msg.data = 3;
	gantry_mode_pub.publish(gantry_mode_msg);
}
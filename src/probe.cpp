#include "probe/probe.h"

#define M_PI 3.14159265358979323846


// Helper functions

// float Probe::calculateProbeExtension(float encoder_counts){
// 	float probe_encoder_count_per_rev = 12;
// 	float probe_gear_ratio = 27;
// 	float probe_lead = 0.008; // [m]
// 	return encoder_counts*probe_lead/(probe_gear_ratio*probe_encoder_count_per_rev);
// }

std::vector<float> Probe::generateSamplingPoints(){
	std::vector<float> pts;
	for(int i = 0; i<target_x.size(); i++){
		float first_sample_point = target_x.at(i) - sample_width/2;
		for(int j = 0; j<num_probes_per_obj; j++){
			float x = first_sample_point+spacing_between_probes*j;
			pts.push_back(x);
		}
	}
	return pts;
}

// Shape classification functions

float Probe::calcRadius(point2D& cc, std::vector<point2D>& points)
{
	float rHat = 0;
	float dx, dy;
	int numPoints = points.size();
	for (int i = 0; i<numPoints; i++){
		dx = points[i].x - cc.x;
		dy = points[i].y - cc.y;
		rHat += sqrt(dx*dx + dy*dy);
	}
	return rHat / numPoints;
}

Probe::point2D Probe::circumcenter(const std::vector<point2D>& points)
{
	float pIx = points[0].x;
	float pIy = points[0].y;
	float pJx = points[1].x;
	float pJy = points[1].y;
	float pKx = points[2].x;
	float pKy = points[2].y;

	point2D dIJ, dJK, dKI;
	dIJ.x = pJx - pIx;
	dIJ.y = pJy - pIy;

	dJK.x = pKx - pJx;
	dJK.y = pKy - pJy;

	dKI.x = pIx - pKx;
	dKI.y = pIy - pKy;

	float sqI = pIx * pIx + pIy * pIy;
	float sqJ = pJx * pJx + pJy * pJy;
	float sqK = pKx * pKx + pKy * pKy;

	float det = dJK.x * dIJ.y - dIJ.x * dJK.y;
	point2D cc;

	if (abs(det) < 1.0e-10)
	{
		cc.x=0;
		cc.y=0;
	}

	cc.x = (sqI * dJK.y + sqJ * dKI.y + sqK * dIJ.y) / (2 * det);
	cc.y = -(sqI * dJK.x + sqJ * dKI.x + sqK * dIJ.x) / (2 * det);

	return cc;
}

Probe::circle Probe::calcCircle(std::vector<point2D>& points)
{
	circle circle;
	point2D cc;
	float sigX = 0;
	float sigY = 0;
	int q = 0;

	int n = points.size();

  for (int i = 0;i<n-2;i++){ // go through all the combinations of points
  	for (int j = i+1;j<n-1;j++){
  		for (int k = j+1;k<n;k++){	// create a vector of three points
  			std::vector<point2D> threePoints;
  			threePoints.push_back(points[i]);
  			threePoints.push_back(points[j]);
  			threePoints.push_back(points[k]);
  			cc = circumcenter(threePoints);
  			sigX += cc.x;
  			sigY += cc.y;
  			q++;
  		}
  	}
  }

  cc.x = sigX/q;
  cc.y = sigY/q;
  circle.center = cc;
  circle.rad = calcRadius(cc, points);
  return circle;
}

Probe::circle Probe::classify(const std::vector<geometry_msgs::PointStamped>& pts3d)
{
	std::vector<point2D> pts2d;
	float x,y;
	for(int i = 0; i < pts3d.size(); i++){
		x = pts3d.at(i).point.x; // project 3d points to 2d
		y = pts3d.at(i).point.y;
		pts2d.push_back({x,y});
	}
	return calcCircle(pts2d);
}



// Initialization functions

void Probe::initializeGantry(){
	// gantry_send_msg.data.clear(); // flush out previous data

	// gantry_send_msg.data.push_back(1); // change it to initialize mode
	// gantry_send_msg.data.push_back(0); // change it to initialize mode

	// gantry_cmd_pub.publish(gantry_send_msg);
	// gantry_init_cmd_sent = true;
}

void Probe::initializeProbes(){
	probe_send_msg.data = probe_initialize_cmd;
	probe_cmd_pub.publish(probe_send_msg);
	probe_init_cmd_sent = true;
}

// Command functions

void Probe::sendGantryPosCmd(){

	ROS_INFO("%f sent to gantry", gantry_pos_cmd);

	gantry_handshake = false;
	gantry_command_arrived = false;

	// ROS_INFO("gantry pos cmd: %d",(int)gantry_pos_cmd*1000);
	gantry_send_msg.data.clear(); // flush out previous data
	// if(gantry_safe_to_move){	
		gantry_send_msg.data.push_back(1); // safe to move
		gantry_send_msg.data.push_back((int)(gantry_pos_cmd*1000)); // input the position [mm]
	// } else {	
	// 	gantry_send_msg.data.push_back(0); // not safe to move
	// 	gantry_send_msg.data.push_back(0); // input the position [mm]
	// }

	gantry_cmd_pub.publish(gantry_send_msg); // send the message
	// gantry_pos_cmd_sent = true; // change the flag
}

void Probe::sendGantryIdleCmd(){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data.push_back(0); // not safe to move
	gantry_send_msg.data.push_back(0); // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
}

void Probe::updateGantryState() {
	// p.gantry_cmd_hack_msg.data.clear(); // flush out previous data
	gantry_cmd_hack_msg.data = 3;
	gantry_cmd_hack_pub.publish(gantry_cmd_hack_msg);
}



void Probe::sendProbeInsertCmd(){
	probe_send_msg.data = probe_insert_cmd; // change it to probe mode
	probe_cmd_pub.publish(probe_send_msg); // send the message
}

bool Probe::sendProbeCmd(int cmd) {
	ROS_INFO("cmd_%d sent to probes", cmd);
	desired_probe_mode = cmd;
	probe_handshake = false;
	probe_command_arrived = false;
	probe_send_msg.data = cmd; // change it to probe mode
	probe_cmd_pub.publish(probe_send_msg); // send the message
}

void Probe::printResults(){
	int mine_correct = 0;
	int mine_incorrect = 0;
	int nonmine_correct = 0;
	int nonmine_incorrect = 0;
	bool guess;
	ROS_INFO("1");
	for (int i = 0; i<num_targets; i++){
		std::vector<geometry_msgs::PointStamped> pts; // (should overwrite with every new i)
		ROS_INFO("2");
		int num_valid_points = 0;
		ROS_INFO("3");



		for (int j = 0; j<num_probes_per_obj; j++){
			ROS_INFO("i = %d, j = %d", i,j); // was ROS_INFO("4");
			bool valid_point = contact_type.at(num_probes_per_obj*i+j);
			if (valid_point){ // if the point is valid (i.e. not at end stops)
				ROS_INFO("5");
				pts.push_back(contact_points.at(num_probes_per_obj*i+j)); // only put valid points in vector
				num_valid_points++;
			}
		}
		ROS_INFO("nvp: %d", num_valid_points);

		if (num_valid_points>=3){ // if you've got enough points to meaningfully classify with
			circle circle = classify(pts);
			(circle.rad<=max_radius) ? guess = true : guess = false; // check against radius threshold
			if(guess){ // if it is a mine, calculate its center and its distance to the actual location
				float dist = sqrt(pow(target_x.at(i)-circle.center.x,2)+pow(target_y.at(i)-circle.center.y,2));
				ROS_INFO("Object %d is a landmine. Calculated center is %fcm away from actual location.", i+1, dist);
				(guess==isMine.at(i)) ? mine_correct++ : mine_incorrect++;
			}
		} else guess = false; // if you didn't hit 3 points then you can't classify it

		// } else if (num_valid_points>0 && num_valid_points<3){ // if you have 1 or 2 points
		// 	ROS_INFO("Object %d is solid but there is not enough information to work out if is a landmine.", i+1);
		// }

		if(!guess)
		{
			ROS_INFO("Object %d is not a landmine.", i+1);
			(guess==isMine.at(i)) ? nonmine_correct++ : nonmine_incorrect++;
		}
	}
	ROS_INFO("%d out of 3 mines were identified correctly.", mine_correct);
	ROS_INFO("%d out of 3 non-mines were identified correctly.", nonmine_correct);
	// demo_complete = true;
	exit;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "probe");
	Probe p;
	ros::Rate loop_rate(10);
	ros::Rate delay(1);
	delay.sleep(); // don't miss first command!

	std::vector<float> sampling_points = p.generateSamplingPoints();
	ROS_INFO("%d targets, each with %d sampling points, for a total of %d probes", 
		p.num_targets, p.num_probes_per_obj, p.num_targets*p.num_probes_per_obj);

	bool blockGantry = false;
	bool blockProbe = false;

	while (ros::ok())
	{
		/*** GANTRY CALIBRATION ***/
		if (p.gantry_initialized == 0) {
			ROS_INFO("Waiting for Gantry to Calibrate...");
		}
		else {
			p.updateGantryState();
			if (p.probe_handshake) p.probe_command_arrived = true;
			if (p.gantry_handshake) p.gantry_command_arrived = true;

			if (p.probe_command_arrived && p.gantry_command_arrived)
			{
				/*** PROBE CALIBRATION ***/
				if (!p.probes_initialized){

					if (p.probe_mode == 1 && !blockGantry) {
						p.gantry_pos_cmd = p.probe_calibration_position;
						p.sendGantryPosCmd();
						blockGantry = true;
						blockProbe = false;
					}
					else if (p.gantry_pos_cmd_reached && !blockProbe) {
						p.sendProbeCmd(3);
						blockProbe = true;
						blockGantry = false;
					}
				}
				/*** NORMAL OPERATION ***/
				else {
					ROS_INFO("command received: %d. P Mode: %d. P Init: %d.", 
						p.probe_command_arrived, p.probe_mode, p.probes_initialized);

					ROS_INFO("G Init: %d. G Mode: %d. G Pos Cmd Reached: %d. G Pos Cmd: %f. G Pos Act: %f", 
						p.gantry_initialized, p.gantry_mode, p.gantry_pos_cmd_reached, p.gantry_pos_cmd, p.gantry_carriage_pos);
					ROS_INFO("After this probe, the next sampling point is: %d", p.sampling_point_index);//, sampling_points.at(p.sampling_point_index));

					if (p.probe_mode == 1 && !blockGantry) {

						p.gantry_pos_cmd = sampling_points.at(p.sampling_point_index);

						p.sampling_point_index++;

						/*** EXIT CONDITION ***/
						if (p.sampling_point_index > (p.total_num_samples-1)) {
							p.sampling_point_index = (p.total_num_samples-1);
							p.printResults();
						}

						p.sendGantryPosCmd();
						blockGantry = true;
						blockProbe = false;
					}

					else if (p.gantry_pos_cmd_reached && !blockProbe) {
						p.sendProbeCmd(2);
						blockProbe = true;
						blockGantry = false;
					}
				}
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}




// vvvv HERE IS THE CODE I HAD ON SATURDAY NIGHT

// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "probe");
// 	Probe p;
// 	ros::Rate loop_rate(10);
// 	ros::Rate delay(1);
// 	delay.sleep();

// 	while (ros::ok())
// 	{

// 		// p.gantry_pos_cmd_actual_delta = (int)p.gantry_carriage_pos-p.gantry_pos_cmd;
// 		switch(p.both_initialized){
// 			case false: // if either one of the gantry or probes has not completed initialization
// 			if (p.gantry_mode==3){ // this becomes true when gantry initialization flag is true // removed else
// 				if(!p.probe_init_cmd_sent){ // if you haven't sent the command to initialize the probes
// 					ROS_INFO("Block 1");
//                     p.sendGantryIdleCmd(); // tell the gantry to stop so that the motor doesn't inadvertently move during probing
// 					p.initializeProbes(); // send the command
// 				}
// 				else if(p.probes_initialized){ // this becomes true when probe initialization flag is true
// 					p.both_initialized = true; // initialization of both is complete
// 					ROS_INFO("Block 2");
// 				}
// 			}
// 			break;
// 			case true: // when both probes and gantry have been initialized
// 			switch(p.full_inspection_complete){
// 				case false: // not all targets have been inspected
// 				if(!p.sampling_points_generated){
// 					sampling_points = p.generateSamplingPoints(p.target_x[p.current_target_id]); // create a vector of points around the current target center
// 					for (int i; i<sampling_points.size();i++){
// 						ROS_INFO("%f",sampling_points.at(i));
// 					}
// 					ROS_INFO("Block 3");
// 				}
// 				if(!p.gantry_pos_cmd_sent){ // if you haven't told the gantry to move to the next position
// 					p.gantry_pos_cmd = 400.0;//(int)(sampling_points[p.sampling_point_index]*1000);
// 					// p.sendGantryPosCmd(); // send the position command
// 					p.gantry_send_msg.data.clear(); // flush out previous data
// 					p.gantry_send_msg.data.push_back(1); // safe to move
// 					// ROS_INFO("%d", pos);
// 					int to_send = (int)p.gantry_pos_cmd;
// 					p.gantry_send_msg.data.push_back(to_send); // input the position [mm]
// 					// gantry_send_msg.data.push_back(gantry_pos_cmd); // input the position [mm]
// 					// gantry_cmd_pub.publish(gantry_send_msg); // send the message
// 					p.gantry_pos_cmd_sent = true; // change the flag
// 					// delay.sleep();
// 					p.sampling_point_index++; // go to the next sampling point
// 					ROS_INFO("Block 4");
// 				}
// 				if(p.gantry_pos_cmd_reached){ // when you've reached the desired position
// 				// if(p.gantry_pos_cmd_actual_delta<=5){ // when you've reached the desired position
// 					p.sendGantryIdleCmd(); // tell the gantry to stop so that the motor doesn't inadvertently move during probing
// 					if(!p.probe_insert_cmd_sent){ // if you haven't told the probes to insert
// 						p.sendProbeInsertCmd(); // tell the probes to insert
// 						ROS_INFO("Block 5");
// 					}
// 					else if(p.probes_returned_home==1){ // if probe is back at home
// 						if(p.sampling_point_index==p.num_probes_per_obj){ // if you've done the specified number of probes per object
// 							// p.indiv_inspection_complete = true; // inspection for this object is complete
// 							p.current_target_id++; // move to the next target ID
// 							p.sampling_point_index = 0;
// 							p.sampling_points_generated = false; // allows new set of sampling points to be generated for next target
// 							ROS_INFO("Block 6");
// 							if(p.current_target_id==p.num_targets){
// 								p.full_inspection_complete = true;
// 								ROS_INFO("Block 7");
// 							} 
// 						}
// 						p.probe_insert_cmd_sent = false; 
// 						p.gantry_pos_cmd_sent = false; // change the flag
// 					}
// 				}
// 				break;
// 				case true: // all targets have been inspected
// 				switch(p.demo_complete){
// 					case false:
// 					p.printResults();
//                     case true: // do nothing
//                     break;
//                     default:
//                     break;
//                 }
//                 break;
//                 default:
//                 break;
//             }
//             break;
//             default:
//             break;
//         }

//         ros::spinOnce();
// 		ROS_INFO("G Init: %d. G Mode: %d. G Pos Cmd Sent: %d. G Pos Cmd Reached: %d. G Pos Cmd: %f. G Pos Act: %f", p.gantry_initialized, p.gantry_mode, p.gantry_pos_cmd_sent, p.gantry_pos_cmd_reached, p.gantry_pos_cmd, p.gantry_carriage_pos);
// 		ROS_INFO("P Init: %d. P Mode: %d. P Ins Cmd Sent: %d. P at Home: %d. P Solid Contact: %d", p.probes_initialized, p.probe_mode, p.probe_insert_cmd_sent, p.probes_returned_home, p.probe_solid_contact );
// 		ROS_INFO("Current target ID: %d. Sampling point ID: %d", p.current_target_id, p.sampling_point_index);
//         loop_rate.sleep();
//     }
//     return 0;
// }



//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_A4139373630351909060-if00
// rosrun rosserial_python serial_node.py /dev/ttyACM0

//probe teensy
// rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_3652290-if00

//gantry teensy
// rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Teensyduino_USB_Serial_3376540-if00

// gantry position testing
// int main(int argc, char **argv)
// {
// 	ros::init(argc, argv, "probe");
// 	Probe p;
// 	ros::Rate loop_rate(10);

// 	double begin = ros::Time::now().toSec();

// 	std::vector<double> move_times;
// 	std::vector<double> move_locations;
// 	move_times = {5,10,15,20};
// 	int number_locations = 4;
// 	// move_locations = {.21,.22,.23,.24};

// 	move_locations = {200,700,453,433};
// 	int test_target_id = 0;
// 	std::vector<float> sampling_points;
// 	bool done = false;

// 	while (ros::ok())
// 	{
// 		double now = ros::Time::now().toSec();
// 		double elapsed = now-begin;
// 		double next_move_time = move_times.at(test_target_id);
// 		if(elapsed>next_move_time && !done){
// 			p.sendGantryPosCmd(move_locations.at(test_target_id));
// 			ROS_INFO("At %f seconds, move to %f. Target #%d",next_move_time,move_locations.at(test_target_id),test_target_id+1);
// 			if (test_target_id==number_locations-1) {
// 				done = true;
// 			}
// 			else {test_target_id++;}
// 		}
//         ros::spinOnce();

//         loop_rate.sleep();
//     }
//     return 0;
// }

// coord transforms not required anymore
	// // gantry
	// p.gantry.setOrigin( tf::Vector3(0.2,-0.2,0.4));
	// p.gantry.setRotation(tf::Quaternion(0,0,0,1));
	// p.br.sendTransform(tf::StampedTransform(p.gantry,ros::Time::now(), "base_link", "gantry"));

	// 		// initialize transforms

	// // gantry carriage
	// p.gantry_carriage.setOrigin( tf::Vector3(0.3,0,0));
	// p.gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
	// p.br.sendTransform(tf::StampedTransform(p.gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage"));

	// // probe rail
	// p.probe_rail.setOrigin( tf::Vector3(0,-0.1,-0.2));
	// tf::Matrix3x3 m_rot;
	// m_rot.setEulerYPR(0,0 , -30*M_PI/180);
	// tf::Quaternion quat; 	// Convert into quaternion
	// m_rot.getRotation(quat);
	// p.probe_rail.setRotation(tf::Quaternion(quat));
	// p.br.sendTransform(tf::StampedTransform(p.probe_rail,ros::Time::now(), "gantry_carriage", "probe_rail"));

	// // probe tip
	// p.probe_tip.setOrigin( tf::Vector3(0,0.4,0));
	// p.probe_tip.setRotation(tf::Quaternion(0,0,0,1));
	// p.br.sendTransform(tf::StampedTransform(p.probe_tip,ros::Time::now(), "probe_rail", "probe_tip"));


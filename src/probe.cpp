#include "probe/probe.h"

#define M_PI 3.14159265358979323846


// Helper functions

// float Probe::calculateProbeExtension(float encoder_counts){
// 	float probe_encoder_count_per_rev = 12;
// 	float probe_gear_ratio = 27;
// 	float probe_lead = 0.008; // [m]
// 	return encoder_counts*probe_lead/(probe_gear_ratio*probe_encoder_count_per_rev);
// }

std::vector<float> Probe::generateSamplingPoints(float center){
	std::vector<float> pts;
	float first_sample_point = center - sample_width/2;
	for(int i = 0; i<num_probes_per_obj; i++){
		float x = first_sample_point+spacing_between_probes*i;
		pts.push_back(x);
	}
	sampling_points_generated = true;
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
	gantry_send_msg.data.clear(); // flush out previous data

	gantry_send_msg.data.push_back(1); // change it to initialize mode
	gantry_send_msg.data.push_back(0); // change it to initialize mode

	gantry_cmd_pub.publish(gantry_send_msg);
	gantry_init_cmd_sent = true;
}

void Probe::initializeProbes(){
	probe_send_msg.data = probe_initialize_cmd;
	probe_cmd_pub.publish(probe_send_msg);
	probe_init_cmd_sent = true;
}

// Command functions

void Probe::sendGantryPosCmd(float pos){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data.push_back(1); // safe to move
	gantry_send_msg.data.push_back((int)(pos*1000)); // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
	gantry_pos_cmd_sent = true; // change the flag
}

void Probe::sendGantryIdleCmd(){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data.push_back(0); // not safe to move
	gantry_send_msg.data.push_back(0); // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
	// gantry_idle_cmd_sent = true; // change the flag
}

void Probe::sendProbeInsertCmd(){
	probe_send_msg.data = probe_insert_cmd; // change it to probe mode
	probe_cmd_pub.publish(probe_send_msg); // send the message
	probe_insert_cmd_sent = true; // change the flag
}

void Probe::printResults(){
    int mine_correct = 0;
    int mine_incorrect = 0;
    int nonmine_correct = 0;
    int nonmine_incorrect = 0;
    for (int i = 0; i<num_targets; i++){
		std::vector<geometry_msgs::PointStamped> pts; // (should overwrite with every new i)
		for (int j = 0; j<num_probes_per_obj; j++){
			pts.push_back(contact_points.at(num_probes_per_obj*i+j));
		}
		circle circle = classify(pts);
        bool guess;
        (circle.rad<=max_radius) ? guess = true : guess = false;
        if(guess)
        {
           float dist = sqrt(pow(target_x.at(i)-circle.center.x,2)+pow(target_y.at(i)-circle.center.y,2));
           ROS_INFO("Object %d is a landmine. Calculated center is %fcm away from actual location.", i+1, dist);
           (guess==isMine.at(i)) ? mine_correct++ : mine_incorrect++;
       }
       else
       {
           ROS_INFO("Object %d is not a landmine.", i+1);
           (guess==isMine.at(i)) ? nonmine_correct++ : nonmine_incorrect++;
       }
   }
   ROS_INFO("%d out of 3 mines were identified correctly.", mine_correct);
   ROS_INFO("%d out of 3 non-mines were identified correctly.", nonmine_correct);
   demo_complete = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "probe");
	Probe p;
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		p.sendProbeInsertCmd();
	// 		// initialize transforms
	// // gantry
	// p.gantry.setOrigin( tf::Vector3(0.2,-0.2,0.4));
	// p.gantry.setRotation(tf::Quaternion(0,0,0,1));
	// p.br.sendTransform(tf::StampedTransform(p.gantry,ros::Time::now(), "base_link", "gantry"));
	
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
	
		// switch(p.both_initialized){
		// 	case false: // if either one of the gantry or probes has not completed initialization
		// 	// if(!p.gantry_init_cmd_sent){ // if you haven't sent the command to initialize the gantry 

		// 	// 	p.initializeGantry(); // send the command
  //  //     }
		// 	if (p.gantry_initialized){ // this becomes true when gantry initialization flag is true // removed else
		// 		if(!p.probe_init_cmd_sent){ // if you haven't sent the command to initialize the probes
  //                   p.sendGantryIdleCmd(); // tell the gantry to stop so that the motor doesn't inadvertently move during probing
		// 			p.initializeProbes(); // send the command
		// 		}
		// 		else if(p.probes_initialized){ // this becomes true when probe initialization flag is true
		// 			p.both_initialized = true; // initialization of both is complete
		// 		}
		// 	}
		// 	break;
		// 	case true: // when both probes and gantry have been initialized
		// 	switch(p.full_inspection_complete){
		// 		case false: // not all targets have been inspected
		// 		if(!p.sampling_points_generated){
		// 			sampling_points = p.generateSamplingPoints(p.target_x[p.current_target_id]); // create a vector of points around the current target center
		// 		}
		// 		if(!p.gantry_pos_cmd_sent){ // if you haven't told the gantry to move to the next position
		// 			p.sendGantryPosCmd(sampling_points[p.sampling_point_index]); // send the position command
		// 			p.sampling_point_index++; // go to the next sampling point
		// 		}
		// 		if(p.gantry_pos_cmd_reached){ // when you've reached the desired position
		// 			p.sendGantryIdleCmd(); // tell the gantry to stop so that the motor doesn't inadvertently move during probing
		// 			if(!p.probe_insert_cmd_sent){ // if you haven't told the probes to insert
		// 				p.sendProbeInsertCmd(); // tell the probes to insert
  //            }
		// 			else if(p.probe_cycle_complete){ // if a single probe insertion sequence is complete
		// 				if(p.sampling_point_index==p.num_probes_per_obj-1){ // if you've done the specified number of probes per object
		// 					p.indiv_inspection_complete = true; // inspection for this object is complete
		// 					p.current_target_id++; // move to the next target ID
		// 					p.sampling_points_generated = false; // allows new set of sampling points to be generated for next target
		// 				}
		// 				p.gantry_pos_cmd_sent = false; // change the flag
		// 			}
		// 		}
		// 		if(p.current_target_id==p.num_targets-1){
		// 			p.full_inspection_complete = true; // num_targets-1 because current_target_id starts at 0
		// 		} 
		// 		break;
		// 		case true: // all targets have been inspected
  //               switch(p.demo_complete){
  //                   case false:
  //                   p.printResults();
  //                   case true: // do nothing
  //                   break;
  //                   default:
  //                   break;
  //               }
  //               break;
  //               default:
  //               break;
  //           }
  //           break;
  //           default:
  //           break;
  //       }

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}

//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_A4139373630351909060-if00
// rosrun rosserial_python serial_node.py /dev/ttyACM0

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

// 	move_locations = {.2,.7,.453,.6};
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
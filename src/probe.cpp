#include <probe/probe.h>

#define M_PI 3.14159265358979323846

std_msgs::Int32 probe_send_msg;
std_msgs::Int32MultiArray gantry_send_msg;

static tf::TransformBroadcaster br;
tf::TransformListener probe_listener;

tf::Transform gantry;
tf::Transform gantry_carriage;
tf::Transform probe_rail;
tf::Transform probe_tip;

// probing parameters
std::vector<double> target_x = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
std::vector<double> target_y = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3};
int num_probes_per_obj = 5;
float spacing_between_probes = 0.02; // [m] = 2cm
float sample_width = (float)num_probes_per_obj*spacing_between_probes;
bool isMine;

int num_targets = target_centers.size();
int current_target_id = 0;
int sampling_point_index = 0;
bool sampling_points_generated = false;
bool indiv_inspection_complete = false;
bool full_inspection_complete = false;

// locations of gantry and probe carriages
float probe_carriage_pos;
float gantry_carriage_pos;

// standard commands
int probe_idle_cmd = 1;
int probe_insert_cmd = 2;
int probe_initialize_cmd = 3;

// initialize modes
int probe_mode = 1; // idle
int gantry_mode = 0; // idle

// initialization flags
bool both_initialized = false; // both probes and gantry

bool gantry_initialized = false;
bool gantry_init_cmd_sent = false;
bool gantry_pos_cmd_sent = false;
bool gantry_pos_cmd_reached = false;

bool probes_initialized = false;
bool probe_init_cmd_sent = false;
bool probe_insert_cmd_sent = false;
bool probe_insertion_complete = false;

// publishers
ros::Publisher probe_contact_pub;
ros::Publisher probe_cmd_pub;
ros::Publisher gantry_cmd_pub;

// subscribers
ros::Subscriber probe_status_sub;
ros::Subscriber probe_contact_sub;
ros::Subscriber gantry_status_sub;

std::vector<geometry_msgs::PointStamped> contact_points;

// Helper functions

float calculateProbeExtension(float encoder_counts){
	float probe_encoder_count_per_rev = 12;
	float probe_gear_ratio = 27;
	float probe_lead = 0.008; // [m]
	return encoder_counts*probe_lead/(probe_gear_ratio*probe_encoder_count_per_rev);
}

std::vector<float> generateSamplingPoints(float center){
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

bool classify()
{

}

circle calcCircle(std::vector<point2D>& points)
{
  circle circle;
  point2D cc;
  float sigX = 0;
  float sigY = 0;
  int q = 0;

  int n = points.size();

  for (int i = 0;i<n-2;i++){ // go through all the combinations of points
    for (int j = i+1;j<n-1;j++){
      for (int k = j+1;k<n;k++){
        // create a vector of three points
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
  // if (q==0)
  //   disp('All points aligned')
  // end
  cc.x = sigX/q;
  cc.y = sigY/q;
  circle.center = cc;
  circle.rad = calcRadius(cc, points);
  return circle;
}

point2D circumcenter(const std::vector<point2D>& points)
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

float calcRadius(point2D& cc, std::vector<point2D>& points)
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

// Callback functions

void probeStatusClbk(const std_msgs::Int32MultiArray& msg){
	probe_mode = msg.data[0]; // first entry is the reported state
	probe_carriage_pos = calculateProbeExtension((float)msg.data[2]); // second entry is the probe carriage position [mm]
	probe_tip.setOrigin( tf::Vector3(0,0.4+probe_carriage_pos,0)); // update origin of probe tip coordinate frame [m]
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
	probes_initialized = (bool)msg.data[1];
	probe_insertion_complete = (bool)msg.data[2];
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
	probe_contact_pub.publish(cp); // publish
	contact_points.push_back(cp); // save into vector
}

void gantryStatusClbk(const std_msgs::Int32MultiArray& msg){
	gantry_mode = msg.data[0]; // first entry is the reported state
	gantry_carriage_pos = (float)msg.data[1]/1000.0; // second entry is the gantry carriage position [mm->m]
	gantry_carriage.setOrigin( tf::Vector3(0,0.1+gantry_carriage_pos,0)); // update origin of gantry carriage coordinate frame
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform
	gantry_initialized = (bool)msg.data[1];
	gantry_pos_cmd_reached = (bool)msg.data[2]; // third entry is the status of whether or not the command position has been reached
}

// Initialization functions

void initializeGantry(){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data[0] = 1; // change it to initialize mode
	gantry_send_msg.data[1] = 0; // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg);
	gantry_init_cmd_sent = true;
}

void initializeProbes(){
	probe_send_msg.data = probe_initialize_cmd;
	probe_cmd_pub.publish(probe_send_msg);
	probe_init_cmd_sent = true;
}

// Command functions

void sendGantryPosCmd(float pos){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data[0] = 3; // change it to position mode
	gantry_send_msg.data[1] = (int)(pos*1000); // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
	gantry_pos_cmd_sent = true; // change the flag
}

void sendGantryIdleCmd(){
	gantry_send_msg.data.clear(); // flush out previous data
	gantry_send_msg.data[0] = 0; // change it to position mode
	gantry_send_msg.data[1] = 0; // input the position [mm]
	gantry_cmd_pub.publish(gantry_send_msg); // send the message
	// gantry_idle_cmd_sent = true; // change the flag
}

void sendProbeInsertCmd(){
	probe_send_msg.data = 2; // change it to probe mode
	probe_cmd_pub.publish(probe_send_msg); // send the message
	probe_insert_cmd_sent = true; // change the flag
}

void printResults(){
	for (int i = 0; i<num_targets; i++){
		// std::vector<float> 
		for (int j = 0; j<num_probes_per_obj; j++){

		}
		isMine = classify();
		if(isMine)
		{
			float dist = 2;
			ROS_INFO("Object %d is a landmine. Center is %fcm away from actual location.", i+1, dist);
		}
		else
		{
			ROS_INFO("Object %d is not a landmine.", i+1);
		}
	}
}

int main(int argc, char **argv)
{
	ros::NodeHandle n;

	ros::init(argc, argv, "probe");
	ros::Rate loop_rate(10);

	// publishers
	probe_cmd_pub = n.advertise<std_msgs::Int32>("probe_cmd_send", 1000);
	probe_contact_pub = n.advertise<geometry_msgs::PointStamped>("probe_contact_send", 1000);
	gantry_cmd_pub = n.advertise<std_msgs::Int32MultiArray>("gantry_cmd_send", 1000);

  	// subscribers
	probe_status_sub = n.subscribe("probe_status_reply", 1000, probeStatusClbk);
	probe_contact_sub = n.subscribe("probe_contact_reply", 1000, probeContactClbk);
	gantry_status_sub = n.subscribe("gantry_status_reply", 1000, gantryStatusClbk);

	// initialize transforms
	// gantry
	gantry.setOrigin( tf::Vector3(0.2,-0.2,0.4));
	gantry.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(gantry,ros::Time::now(), "base_link", "gantry"));
	
	// gantry carriage
	gantry_carriage.setOrigin( tf::Vector3(0,0.1,0));
	gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage"));

	// probe rail
	probe_rail.setOrigin( tf::Vector3(0,-0.1,-0.2));
	tf::Matrix3x3 m_rot;
	m_rot.setEulerYPR(0, -30*M_PI/180, 0);
	tf::Quaternion quat; 	// Convert into quaternion
	m_rot.getRotation(quat);
	probe_rail.setRotation(tf::Quaternion(quat));
	br.sendTransform(tf::StampedTransform(probe_rail,ros::Time::now(), "gantry_carriage", "probe_rail"));

	// probe tip
	probe_tip.setOrigin( tf::Vector3(0,0.4,0));
	probe_tip.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip"));
	
	std::vector<float> sampling_points;

	while (ros::ok())
	{
		switch(both_initialized){
			case false: // if either one of the gantry or probes has not completed initialization
			if(!gantry_init_cmd_sent) // if you haven't sent the command to initialize the gantry
			{ 
				initializeGantry(); // send the command
			}
			else if (gantry_initialized){ // this becomes true when gantry initialization flag is true
				if(!probe_init_cmd_sent) // if you haven't sent the command to initialize the probes
				{ 
					initializeProbes(); // send the command
				}
				else if(probes_initialized) // this becomes true when probe initialization flag is true
				{ 
					both_initialized = true; // initialization of both is complete
				}
			}
			break;
			case true: // when both probes and gantry have been initialized
			switch(full_inspection_complete){
				case false: // not all targets have been inspected
				if(!sampling_points_generated)
				{
					sampling_points = generateSamplingPoints(target_centers[current_target_id]); // create a vector of points around the current target center
				}
				if(!gantry_pos_cmd_sent) // if you haven't told the gantry to move to the next position
				{ 
					sendGantryPosCmd(sampling_points[sampling_point_index]); // send the position command
					sampling_point_index++; // go to the next sampling point
				}
				if(gantry_pos_cmd_reached) // when you've reached the desired position
				{ 
					sendGantryIdleCmd(); // tell the gantry to stop so that the motor doesn't inadvertently move during probing
					if(!probe_insert_cmd_sent) // if you haven't told the probes to insert
					{ 
						sendProbeInsertCmd(); // tell the probes to insert
					}
					else if(probe_insertion_complete) // if the probe insertion sequence is complete
					{
						if(sampling_point_index==num_probes_per_obj-1) // if you've done the specified number of probes per object
						{
							indiv_inspection_complete = true; // inspection for this object is complete
							current_target_id++; // move to the next target ID
							sampling_points_generated = false; // allows new set of sampling points to be generated for next target
						}
						gantry_pos_cmd_sent = false; // change the flag
					}
				}

				if(current_target_id==num_targets-1)
				{
					full_inspection_complete = true; // num_targets-1 because current_target_id starts at 0
				} 
				break;
				case true: // all targets have been inspected
				printResults();
				break;
				default:
				break;
			}
			break;
			default:
			break;
		}


		// ROS_INFO("Probe Cmd: %d --- Probe Mode: %d ------ Gantry Cmd: %d --- Gantry Mode: %d",probe_cmd,probe_mode,gantry_cmd,gantry_mode);
		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

//rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/serial/by-id/usb-Arduino_Srl_Arduino_Uno_8543130373635161B1C0-if00
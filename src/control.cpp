
#include "probe.h"
#include "gantry.h"
#include "probe/mine.h"

int control_state = 0; // TODO: implement this for cleaner internal state loop
#define GANTRY_CALIBRATION			0
#define GANTRY_TO_PROBE_CALIBRATION	1
#define PROBE_CALIBRATION 			2
#define PROBING 					3
#define MOVING_GANTRY				4
	
#define M_PI 3.14159265358979323846

const float probe_calibration_position = 0.05f; // i.e calibrate probes at 50cm

									//	test1	test2	test3	test4 	test5	test6	[m]
std::vector<double> target_x = 		{	0.08, 	0.16, 	0.24,	0.365,	0.525,	0.695};
std::vector<double> target_y = 		{	0.34,	0.34,	0.34,	0.36,	0.365,	0.365};
std::vector<double> target_rad = 	{	0,		0,		0,		0.0575,	0.075,	0.0575};
std::vector<bool> target_truth = 	{	false,	false,	false, 	true, 	true, 	true};
std::vector<int> target_samples = 	{	3,		3,		3, 		6, 		10, 		6};

const float spacing_between_probes = 0.015; // [m]
int sampling_point_index;
int num_probes_per_obj;
float sample_width;

std::vector<float> newLandmine(double _x, int _samples) {

	num_probes_per_obj = _samples;
	sample_width = (num_probes_per_obj-1)*spacing_between_probes;

	sampling_point_index = 0;

	std::vector<float> pts;

	float first_sample_point = _x - sample_width/2;
	for(int j = 0; j<num_probes_per_obj; j++){
		float x = first_sample_point+spacing_between_probes*j;
		pts.push_back(x);
	}
	return pts;
}

ros::Publisher new_mine_data;
std::vector<float> sampling_points;
int landmine_index = 0;

void probeNextLandmine() {

	probe::mine mine;
	mine.exists = target_truth.at(landmine_index);
	mine.x = target_x.at(landmine_index);
	mine.y = target_y.at(landmine_index);
	mine.radius = target_rad.at(landmine_index);
	new_mine_data.publish(mine);

	sampling_points = newLandmine(target_x.at(landmine_index), 
								  target_samples.at(landmine_index));

	ROS_INFO("%lu probe points generated for target at x = %f, y = %f", 
		sampling_points.size(),target_x.at(landmine_index),target_y.at(landmine_index)); 

	landmine_index++;
}

float goodness_of_fit;
const float goodness_of_fit_thresh = 5.0f;

void estMineClbk(const probe::mine& msg) {
	 goodness_of_fit = msg.goodness_of_fit; // use this information to stopping condition
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "probe");

	Probe probe;
	Gantry gantry;

	ros::NodeHandle n;
	new_mine_data = n.advertise<probe::mine>("new_mine_data", 1000);
	ros::Subscriber mine_estimate = n.subscribe("mine_estimate_data", 1000, &estMineClbk);

	ros::Rate loop_rate(10);
	ros::Rate delay(0.2);
	delay.sleep(); // don't miss first command!

	probeNextLandmine();

	bool blockGantry = false;
	bool blockProbe = false;
	bool finished = false;

	while (ros::ok() && !finished)
	{
		/*** GANTRY CALIBRATION ***/
		if (!gantry.initialized) {
			ROS_INFO("Waiting for Gantry to Calibrate...");
		}
		else {
			gantry.updateState();

			if (probe.notWaiting() && gantry.notWaiting())
			{
				/*** PROBE CALIBRATION ***/
				if (!probe.initialized){

					if (probe.mode == 1 && !blockGantry) {
						ROS_INFO("Waiting for Gantry to move to Calibration Position @ %f", probe_calibration_position);
						gantry.sendPosCmd(probe_calibration_position);
						blockGantry = true;
						blockProbe = false;
					}
					else if (gantry.pos_cmd_reached && !blockProbe) {
						ROS_INFO("Calibrating Probes...");
						probe.sendCmd(3);
						blockProbe = true;
						blockGantry = false;
					}
				}
				/*** NORMAL OPERATION ***/
				else {
					if (probe.mode == 1 && !blockGantry) {

						/*** FINISHED ONE ***/

						ROS_INFO("%d >= %d", sampling_point_index, num_probes_per_obj);

						if (sampling_point_index >= num_probes_per_obj) {
						// if (goodness_of_fit > goodness_of_fit_thresh) { //TODO

							/*** FINISHED ALL ***/
							if (landmine_index == target_x.size()) {
								finished = true;
							}
							else {
								ROS_INFO("New Landmine...");
								probeNextLandmine();
								delay.sleep(); // wait 1 second
							}
						}
						else {

							// TODO: get new sample point!
							ROS_INFO("Sending Gantry to %f", sampling_points.at(sampling_point_index));
							gantry.sendPosCmd(sampling_points.at(sampling_point_index));
							sampling_point_index++;

							blockGantry = true;
							blockProbe = false;
						}
					}

					else if (gantry.pos_cmd_reached && !blockProbe) {
						ROS_INFO("Probing...");
						probe.sendCmd(2);
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


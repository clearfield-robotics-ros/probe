
#include <probe.h>
#include <gantry.h>
#include <landmine_classifier.h>

#define M_PI 3.14159265358979323846

std::vector<double> target_x = {.2,.6}; // {.2,.375,.535,0.695}; // [m]
std::vector<double> target_y = {.34,.34}; //{.34,.34,.34,.34};

int main(int argc, char **argv) {

	ros::init(argc, argv, "probe");
	Probe probe;
	Gantry gantry;
	Landmine_Classifier mine;

	ros::Rate loop_rate(10);
	ros::Rate delay(1);
	delay.sleep(); // don't miss first command!



	int landmine_index = 0;
	std::vector<float> sampling_points = 
		mine.generateSamplingPoints(target_x.at(landmine_index), 
									target_y.at(landmine_index));
	ROS_INFO("%lu probe points generated for target at x = %f, y = %f", 
		sampling_points.size(),target_x.at(landmine_index),target_y.at(landmine_index)); 

	bool blockGantry = false;
	bool blockProbe = false;
	bool finished = false;

	while (ros::ok() && !finished)
	{
		/*** GANTRY CALIBRATION ***/
		if (gantry.initialized == 0) {
			ROS_INFO("Waiting for Gantry to Calibrate...");
		}
		else {
			gantry.updateGantryState();

			if (probe.notWaiting() && gantry.notWaiting())
			{
				/*** PROBE CALIBRATION ***/
				if (!probe.initialized){

					if (probe.mode == 1 && !blockGantry) {
						ROS_INFO("Waiting for Gantry to move to Calibration Position @ %f", gantry.probe_calibration_position);
						gantry.pos_cmd = gantry.probe_calibration_position;
						gantry.sendGantryPosCmd();
						blockGantry = true;
						blockProbe = false;
					}
					else if (gantry.pos_cmd_reached && !blockProbe) {
						ROS_INFO("Calibrating Probes...");
						probe.sendProbeCmd(3);
						blockProbe = true;
						blockGantry = false;
					}
				}
				/*** NORMAL OPERATION ***/
				else {
					if (probe.mode == 1 && !blockGantry) {

						/*** FINISHED ONE ***/
						if (mine.sampling_point_index >= mine.num_probes_per_obj) {

							mine.printResults();
							delay.sleep(); // wait 1 second

							landmine_index++;

							if (landmine_index < target_x.size()){
								sampling_points = mine.generateSamplingPoints(target_x.at(landmine_index), 
																			  target_y.at(landmine_index)); 
								ROS_INFO("%lu probe points generated", sampling_points.size()); 
							}
							/*** FINISHED ALL ***/
							else {
								ROS_INFO("Finished probing all objects..."); 
								finished = true;
							}
						}
						else {
							gantry.pos_cmd = sampling_points.at(mine.sampling_point_index);
							mine.sampling_point_index++;
							ROS_INFO("Sending Gantry to %f", gantry.pos_cmd);
							gantry.sendGantryPosCmd();
							blockGantry = true;
							blockProbe = false;
						}
					}

					else if (gantry.pos_cmd_reached && !blockProbe) {
						ROS_INFO("Probing...");
						probe.sendProbeCmd(2);
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


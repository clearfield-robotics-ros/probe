
#include <probe.h>
#include <gantry.h>
#include <landmine_classifier.h>

#define M_PI 3.14159265358979323846

void ros_debug () {
	// ROS_INFO("command received: %d. P Mode: %d. P Init: %d.", 
	// 	p.probe_command_arrived, p.probe_mode, p.probes_initialized);

	// ROS_INFO("G Init: %d. G Mode: %d. G Pos Cmd Reached: %d. G Pos Cmd: %f. G Pos Act: %f", 
	// 	p.gantry_initialized, p.gantry_mode, p.gantry_pos_cmd_reached, p.gantry_pos_cmd, p.gantry_carriage_pos);
	// ROS_INFO("After this probe, the next sampling point is: %d", p.sampling_point_index);//, sampling_points.at(p.sampling_point_index));
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "probe");
	Probe probe;
	Gantry gantry;
	Landmine_Classifier mine;

	ros::Rate loop_rate(10);
	ros::Rate delay(1);
	delay.sleep(); // don't miss first command!

	std::vector<float> sampling_points = mine.generateSamplingPoints();

	ROS_INFO("%d targets, each with %d sampling points, for a total of %d probes", 
		mine.num_targets, mine.num_probes_per_obj, mine.num_targets*mine.num_probes_per_obj);

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

					// ros_debug();

					if (probe.mode == 1 && !blockGantry) {

						/*** EXIT CONDITION ***/
						if (mine.sampling_point_index >= (mine.total_num_samples)) {
							mine.printResults();
							finished = true;
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


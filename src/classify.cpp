
#include "classify.h"

Classify::Classify()
{
	vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	mine_estimate_pub = n.advertise<probe::mine>("mine_estimate_data", 1000);

	new_mine_data = n.subscribe("new_mine_data", 1000, &Classify::newMineClbk, this);
	probe_status_sub = n.subscribe("probe_t/probe_status_reply", 1000, &Classify::probeStatusClbk, this);
	probe_contact_sub = n.subscribe("probe_t/probe_contact_reply", 1000, &Classify::probeContactClbk, this);
	gantry_status_sub = n.subscribe("gantry/gantryStat", 1000, &Classify::gantryStatusClbk, this);
}

void Classify::newMineClbk(const probe::mine& msg) {

	ROS_INFO("New Info Recieved!");

	if (!contact.empty()) printResults();

	contact.clear();
	mine = Mine(); // reset
	landmineCount++;
	mine.truth.exists 	= msg.exists;
	mine.truth.x 		= msg.x;
	mine.truth.y 		= msg.y;
	mine.truth.radius 	= msg.radius;
}

void Classify::gantryStatusClbk(const std_msgs::Int16MultiArray& msg) {
    gantry_carriage_pos = (float)msg.data.at(3)/1000.0; // fourth entry is the gantry carriage position [mm->m]    

    gantry_carriage.setOrigin( tf::Vector3(0.09+gantry_carriage_pos,-0.13,0.365)); // update origin of gantry carriage coordinate frame
	gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform}
}

void Classify::probeStatusClbk(const std_msgs::Int16MultiArray& msg){
	probe_carriage_pos = (float)msg.data.at(3)/1000;

	probe_tip.setOrigin( tf::Vector3(0,0.075+probe_carriage_pos,0));
	probe_tip.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
}

void Classify::probeContactClbk(const std_msgs::Int16MultiArray& msg){
	probe_carriage_pos = (float)msg.data.at(3)/1000; // second entry is the probe carriage position [mm]
	solid_contact = (bool)msg.data.at(4);

	probe_tip.setOrigin( tf::Vector3(0,0.075+probe_carriage_pos,0)); // update origin of probe tip coordinate frame
	probe_tip.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform	
	tf::StampedTransform probe_tf;
	probe_listener.lookupTransform("base_link","probe_tip",ros::Time(0),probe_tf);
	geometry_msgs::PointStamped cp; // single contact point (cp)
	tf::Vector3 probe_tip_origin;
	probe_tip_origin = probe_tf.getOrigin();
	cp.point.x = probe_tip_origin.x();
	cp.point.y = probe_tip_origin.y();
	cp.point.z = probe_tip_origin.z();

	/*** CLASSIFICATION ***/
	ROS_INFO("Contact X: %fm. Contact Y: %fm. Contact Z: %fm, Object Found?: %d",
		cp.point.x, cp.point.y, cp.point.z, solid_contact);

	Contact nc;
	nc.point = cp;
	nc.type = solid_contact;
	contact.push_back(nc); // save into vector for internal processing

	calulateResults();	
}

void Classify::calulateResults() {
	bool guess;

	std::vector<geometry_msgs::PointStamped> pts; // (should overwrite with every new i)
	int num_valid_points = 0;

	for (int j = 0; j < contact.size(); j++) {

		if (contact.at(j).type){ // if the point is valid (i.e. not at end stops)
			pts.push_back(contact.at(j).point); // only put valid points in vector
			num_valid_points++;
		}
	}
	ROS_INFO("Number of valid points: %d", num_valid_points);

	if (num_valid_points >= 3) { // if you've got enough points to meaningfully classify with
		
		circle circle = classify(pts);
		guess = true; // crude but working

		mine.estimate.exists 	= guess;
		mine.estimate.x 		= circle.center.x - 0.2f;
		mine.estimate.y 		= circle.center.y;
		mine.estimate.radius 	= circle.rad;

		mine_estimate.exists 	= guess;
		mine_estimate.x 		= circle.center.x - 0.2f;
		mine_estimate.y 		= circle.center.y;
		mine_estimate.radius 	= circle.rad;
		mine_estimate.goodness_of_fit = circle.goodness_of_fit;

		visualizeLandmineEstimate();
	}
	else { // if you didn't hit 3 points then you can't classify it
		guess = false;

		mine_estimate.exists 	= guess;
		mine_estimate.x 		= 0.0f;
		mine_estimate.y 		= 0.0f;
		mine_estimate.radius 	= 0.0f;
		mine_estimate.goodness_of_fit = 0.0f;
	}
	mine_estimate_pub.publish(mine_estimate);	
}

void Classify::printResults() {

	printf("\n\t--------------------------------\n");
	printf("\tTARGET #%d\n",landmineCount);
	printf("\t--------------------------------\n");

	printf("\tMEASURED TRUTH:\n");

	if (!mine.truth.exists) {
		printf("\tLandmine: False");
	}
	else {
		printf("\tLandmine: True\n");
		printf("\tX: %f, Y: %f, Radius: %f", 
			mine.truth.x,
			mine.truth.y,
			mine.truth.radius);
	}

	printf("\n\tESTIMATED VALUES:\n");

	if (!mine.estimate.exists) {
		printf("\tLandmine: False");
	}
	else {
		printf("\tLandmine: True\n");
		printf("\tX: %f, Y: %f, Radius: %f", 
			mine.estimate.x,
			mine.estimate.y,
			mine.estimate.radius);
	}

	printf("\n\t--------------------------------\n\n");
}

void Classify::visualizeLandmineEstimate() {
	visualization_msgs::Marker marker;
	marker.header.frame_id = "base_link";
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = landmineCount;
	marker.type = visualization_msgs::Marker::CYLINDER;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = mine.estimate.x;
	marker.pose.position.y = mine.estimate.y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0; 
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 2*mine.estimate.radius;
	marker.scale.y = 2*mine.estimate.radius;
	marker.scale.z = 0.05;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0;
	marker.color.g = 0.5;
	marker.color.b = 0.5;
	//only if using a MESH_RESOURCE marker type:
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	vis_pub.publish( marker );
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "classify");

	Classify classify;
	ROS_INFO("Classification node up & running!");

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	} 

}
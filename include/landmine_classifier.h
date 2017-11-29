
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "geometry_msgs/PointStamped.h"

#include <visualization_msgs/Marker.h>

class Landmine_Classifier
{
public:

	struct Landmine {
		bool mine_truth;
		float x_truth;
		float y_truth;
		float radius_truth;
		bool mine_est;
		float x_est;
		float y_est;
		float radius_est;
	};

	struct point2D { float x, y; };

	struct circle {
	    point2D center;
	    float rad;
	};

	Landmine landmine;

	int sampling_point_index;
	const int num_probes_per_obj = 8;
	const float spacing_between_probes = 0.01; // [m]
	const float sample_width = (num_probes_per_obj-1)*spacing_between_probes;
	const float max_radius = 0.1; // [m] = 15cm
	const int num_targets = 1;

	// locations of gantry and probe carriages
	float carriage_pos;
	bool initialized	 	= false;
	bool solid_contact 		= false;

	ros::NodeHandle n;

	// subscribers
	ros::Subscriber gantry_status_sub;
	ros::Subscriber probe_status_sub;
	ros::Subscriber probe_contact_sub;

	ros::Publisher probe_contact_pub;

	// transforms
	tf::TransformBroadcaster br;

	tf::Transform gantry;
	tf::Transform gantry_carriage;

	tf::TransformListener probe_listener;
	tf::Transform probe_rail;
	tf::Transform probe_tip;

	std::vector<geometry_msgs::PointStamped> contact_points;
	std::vector<bool> contact_type;

	Landmine_Classifier()
	{
		probe_status_sub = n.subscribe("probe_t/probe_status_reply", 1000, &Landmine_Classifier::probeStatusClbk, this);
		probe_contact_sub = n.subscribe("probe_t/probe_contact_reply", 1000, &Landmine_Classifier::probeContactClbk, this);
		gantry_status_sub = n.subscribe("gantry/gantryStat", 1000, &Landmine_Classifier::gantryStatusClbk, this);
	};

	void gantryStatusClbk(const std_msgs::Int16MultiArray& msg) {
	    carriage_pos = (float)msg.data.at(3)/1000.0; // fourth entry is the gantry carriage position [mm->m]    
	    gantry_carriage.setOrigin( tf::Vector3(0.09+carriage_pos,-0.13,0.365)); // update origin of gantry carriage coordinate frame
		gantry_carriage.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(gantry_carriage,ros::Time::now(), "gantry", "gantry_carriage")); // broadcast probe tip transform}
	}

	void probeStatusClbk(const std_msgs::Int16MultiArray& msg){
		carriage_pos = (float)msg.data.at(3)/1000;
		probe_tip.setOrigin( tf::Vector3(0,0.075+carriage_pos,0));
		probe_tip.setRotation(tf::Quaternion(0,0,0,1));
		br.sendTransform(tf::StampedTransform(probe_tip,ros::Time::now(), "probe_rail", "probe_tip")); // broadcast probe tip transform
	}

	void probeContactClbk(const std_msgs::Int16MultiArray& msg){
		initialized = (bool)msg.data.at(1);
		if(!initialized){ // if you got here by accident during initialization, don't record the point
			return;
		}
		else {
			carriage_pos = (float)msg.data.at(3)/1000; // second entry is the probe carriage position [mm]
			solid_contact = (bool)msg.data.at(4);

			// ROS_INFO("Probe carriage pos: %f. Probe solid contact: %d", probe_carriage_pos, probe_solid_contact);
			probe_tip.setOrigin( tf::Vector3(0,0.075+carriage_pos,0)); // update origin of probe tip coordinate frame
			
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

			// probe_contact_pub.publish(cp); // publish to save in rosbag - CAUSING CRASH?

			/*** CLASSIFICATION ***/
			ROS_INFO("Contact X: %fm. Contact Y: %fm. Contact Z: %fm, Object Found?: %d",
				cp.point.x, cp.point.y, cp.point.z, solid_contact);
			contact_points.push_back(cp); // save into vector for internal processing
			contact_type.push_back(solid_contact);
		}
	}


	/*** PLANNING NEW PROBES ***/

	std::vector<float> newLandmine(float _x, float _y, float _radius, bool _mine){

		contact_points.clear();
		contact_type.clear();
		landmine = Landmine(); // reset

		landmine.mine_truth = _mine;
		landmine.x_truth = _x;
		landmine.y_truth = _y;
		landmine.radius_truth = _radius;

		sampling_point_index = 0;

		std::vector<float> pts;

		float first_sample_point = _x - sample_width/2;
		for(int j = 0; j<num_probes_per_obj; j++){
			float x = first_sample_point+spacing_between_probes*j;
			pts.push_back(x);
		}

		return pts;
	}

	/*** CLASSIFICATION ***/

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

	circle classify(const std::vector<geometry_msgs::PointStamped>& pts3d)
	{
		std::vector<point2D> pts2d;
		float x,y;
		for(int i = 0; i < pts3d.size(); i++){
			x = pts3d.at(i).point.x; // project 3d points to 2d
			y = pts3d.at(i).point.y;
			ROS_INFO("x:%f y:%f",x,y);
			pts2d.push_back({x,y});
		}
		return calcCircle(pts2d);
	}

	void calulateResults(int landmineCount) {
		bool guess;

		std::vector<geometry_msgs::PointStamped> pts; // (should overwrite with every new i)
		int num_valid_points = 0;

		for (int j = 0; j<num_probes_per_obj; j++) {

			if (contact_type.at(j)){ // if the point is valid (i.e. not at end stops)
				pts.push_back(contact_points.at(j)); // only put valid points in vector
				num_valid_points++;
			}
		}
		ROS_INFO("Number of valid points: %d", num_valid_points);

		if (num_valid_points>=3) { // if you've got enough points to meaningfully classify with
			circle circle = classify(pts);
			(circle.rad<=max_radius) ? guess = true : guess = false; // check against radius threshold

			ROS_INFO("Calculated center at: X = %fm. Y = %fm. Radius = %fm", circle.center.x, circle.center.y, circle.rad);

			landmine.mine_est = guess;	// Assign estimates to landmine object
			landmine.x_est = circle.center.x;
			landmine.y_est = circle.center.y;
			landmine.radius_est = circle.rad;
		} 
		else guess = false; // if you didn't hit 3 points then you can't classify it

		printResults(landmineCount);
	}

	void printResults(int landmineCount) {

		printf("\n\t--------------------------------\n");
		printf("\tTARGET #%d\n",landmineCount);
		printf("\t--------------------------------\n");

		printf("\tMEASURED TRUTH:\n");

		if (!landmine.mine_truth) {
			printf("\tLandmine: False");
		}
		else {
			printf("\tLandmine: True\n");
			printf("\tX: %f, Y: %f, Radius: %f", 
				landmine.x_truth,
				landmine.y_truth,
				landmine.radius_truth);
		}

		printf("\n\tESTIMATED VALUES:\n");

		if (!landmine.mine_est) {
			printf("\tLandmine: False");
		}
		else {
			printf("\tLandmine: True\n");
			printf("\tX: %f, Y: %f, Radius: %f", 
				landmine.x_est,
				landmine.y_est,
				landmine.radius_est);
		}

		printf("\n\t--------------------------------\n\n");
	}

private:

};
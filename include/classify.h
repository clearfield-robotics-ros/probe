
#include "ros/ros.h"
#include "probe/mine.h"
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
#include "visualization_msgs/Marker.h"

class Classify
{

public:

	Classify();
	
private:

	struct MineData {
		bool exists;
		float x, y, radius;
	};

	struct Mine {
		MineData truth, estimate;
	};

	struct point2D { 
		float x, y; 
	};

	struct circle {
	    point2D center;
	    float rad;
	    float goodness_of_fit;
	};

	struct Contact {
		geometry_msgs::PointStamped point;
		bool type;
	};

	Mine mine;
	int landmineCount = 0;
	std::vector<Contact> contact;

	// locations of gantry and probe carriages
	float probe_carriage_pos, gantry_carriage_pos;
	bool initialized	 	= false;
	bool solid_contact 		= false;

	ros::NodeHandle n;

	// subscribers
	ros::Subscriber new_mine_data;
	ros::Subscriber gantry_status_sub;
	ros::Subscriber probe_status_sub;
	ros::Subscriber probe_contact_sub;

	// messages
	probe::mine mine_estimate;

	// publishers
	ros::Publisher mine_estimate_pub;
	ros::Publisher probe_contact_pub;
	ros::Publisher vis_pub;

	// transforms
	tf::TransformBroadcaster br;
	tf::Transform gantry;
	tf::Transform gantry_carriage;
	tf::TransformListener probe_listener;
	tf::Transform probe_rail;
	tf::Transform probe_tip;

	void newMineClbk(const probe::mine& msg);

	void gantryStatusClbk(const std_msgs::Int16MultiArray& msg);

	void probeStatusClbk(const std_msgs::Int16MultiArray& msg);

	void probeContactClbk(const std_msgs::Int16MultiArray& msg);

	void calulateResults();

	void printResults();

	void visualizeLandmineEstimate();

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

};

#ifndef CIRCLE_FIT_H
#define CIRCLE_FIT_H

#include "circle_type.h"
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

class Circle_Fit : public Circle_Type
{
public:


	
	Circle_Fit() {};

	circle find(const std::vector<geometry_msgs::PointStamped>& pts3d)
	{
		std::vector<point2D> pts2d;
		float x,y;
		for(int i = 0; i < pts3d.size(); i++){
			x = pts3d.at(i).point.x; // project 3d points to 2d
			y = pts3d.at(i).point.y;
			// ROS_INFO("x:%f y:%f",x,y);
			pts2d.push_back({x,y});
		}
		return calcCircle(pts2d);
	}

public:



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
};

#endif
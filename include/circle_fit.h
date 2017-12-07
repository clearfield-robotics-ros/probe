
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
		// return calcCircle(pts2d);
		// return initCircle(pts2d);
		circle c = initCircle(pts2d); // get a first estimate for the center, to facilitate optimization
		costParams params = computeCost(c, pts2d); // compute cost function of initial guess for center
		return optimizeCircle(params, c, pts2d); // calculate the least-squares fit to the given points
	}

public:

	struct costParams{
		float J, dJdx, dJdy;
	};

	float calcRadius(const point2D& cc, const std::vector<point2D>& points)
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
		point2D cc; // circumcenter

		if (abs(det) < 1.0e-10)
		{
			cc.x=0;
			cc.y=0;
		}

		cc.x = (sqI * dJK.y + sqJ * dKI.y + sqK * dIJ.y) / (2 * det);
		cc.y = -(sqI * dJK.x + sqJ * dKI.x + sqK * dIJ.x) / (2 * det);

		return cc;
	}

	// circle calcCircle(const std::vector<point2D>& points)
	circle initCircle(const std::vector<point2D>& points)
	{
		circle c;
		point2D center_coords;
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
					center_coords = circumcenter(threePoints);
					sigX += center_coords.x;
					sigY += center_coords.y;
					q++;
				}
			}
		}
		center_coords.x = sigX/q;
		center_coords.y = sigY/q;
		c.center = center_coords;
		c.rad = calcRadius(center_coords, points);
		return c; // full circle properties
	}

	costParams computeCost( const circle& c, const std::vector<point2D>& points )
	{
		float J = 0;
		float dJdx = 0;
		float dJdy = 0;

		for (int i = 0; i<points.size(); i++){
			float dx = points.at(i).x - c.center.x;
			float dy = points.at(i).y - c.center.y;
			float di = sqrt(pow(dx,2) + pow(dy,2));
			float dr    = di - c.rad;
			float ratio = dr / di;
			J    = J + dr * (di + c.rad);
			dJdx = dJdx + dx * ratio;
			dJdy = dJdy + dy * ratio;
		}
		dJdx = dJdx*2.0;
		dJdy = dJdy*2.0;

		costParams params; // assign the local variables to the struct
		params.J = J;
		params.dJdx = dJdx;
		params.dJdy = dJdy;
		return params;
	}

	float newtonStep( const float u, const float v, const circle& c, const std::vector<point2D>& points){

		float sum1 = 0;
		float sum2 = 0;
		float sumFac = 0;
		float sumFac2R = 0;

		for (int i = 0; i<points.size(); i++){
			float dx     = c.center.x - points.at(i).x;
			float dy     = c.center.y - points.at(i).y;
			float di     = sqrt(pow(dx,2) + pow(dy,2));
			float coeff1 = (dx * u + dy * v) / di;
			float coeff2 = di - c.rad;
			sum1         = sum1 + coeff1 * coeff2;
			sum2         = sum2 + coeff2 / di;
			sumFac       = sumFac + coeff1;
			sumFac2R     = sumFac2R + pow(coeff1,2) / di;
		}
		float stepLength = -sum1 / ((pow(u,2) + pow(v,2)) * sum2 - pow(sumFac,2) / points.size() + c.rad * sumFac2R);
		return stepLength;
	}

	circle optimizeCircle(const costParams& params, const circle& c, const std::vector<point2D>& points){ // gets passed initial values

		float epsinner = 0.1;
		float epsouter = 1e-10;

		float J = params.J;			// initialize local variables to the ones we were provided with
		float dJdx = params.dJdx;
		float dJdy = params.dJdy;

		circle c_opt;
		c_opt.center.x = c.center.x;
		c_opt.center.y = c.center.y;
		c_opt.rad = c.rad;

		float Jprev = J;
		float prevDJdx = dJdx;
		float prevDJdy = dJdy;
		float uprev = 0;
		float vprev = 0;

		int i = 1; // num iterations
		int max_iter = 100; // max num iterations
		bool converged = false;
		float Jinner, beta, u, v;

		if(!converged && i<=max_iter){

			// search direction
			u = -dJdx;
			v = -dJdy;

			if (i>1)
			{
				beta = (dJdx * (dJdx - prevDJdx) + dJdy * (dJdy - prevDJdy))/ (pow(prevDJdx,2) + pow(prevDJdy,2));
				u = u+beta*uprev;
				v = v+beta*vprev;
			}

			Jprev = J; // where is this redefined??
			prevDJdx = dJdx;
			prevDJdy = dJdy;
			uprev    = u;
			vprev    = v;

			if (i==1) Jinner = J*10; // to make sure it enters inner loop
			
			// minimize along search direction
			while (i<=max_iter && (abs(J-Jinner)/J)>=epsinner)
			{
				Jinner = J;
				float lambda = newtonStep(u,v,c_opt,points);
				c_opt.center.x += lambda*u;
				c_opt.center.y += lambda*v;
				point2D circumcenter;
				circumcenter.x = c_opt.center.x;
				circumcenter.y = c_opt.center.y;				
				c_opt.rad = calcRadius(circumcenter, points);
				costParams params_updated = computeCost( c_opt, points );
				J = params_updated.J;
				dJdx = params_updated.dJdx;
				dJdy = params_updated.dJdy;
			}

			if((abs(J-Jprev)/J)<epsouter)
			{
				converged = true;
			}
			// ROS_INFO("Iteration #%d. Converged: %d",i, converged);
			i++;
		}
		return c_opt;
	}
};

#endif
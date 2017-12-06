
#ifndef CIRCLE_TYPE_H
#define CIRCLE_TYPE_H

class Circle_Type
{
protected:

	struct point2D { 
		float x, y; 
	};

	struct circle {
	    point2D center;
	    float rad;
	    float goodness_of_fit;
	};
};

#endif
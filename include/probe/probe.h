#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include "geometry_msgs/PointStamped.h"

class Probe
{
public:
	Probe();
	~Probe();

private:
	float probe_encoder_count_per_rev = 12;
	float probe_gear_ratio = 27;
	float probe_lead = 0.008; // [m]
	
};


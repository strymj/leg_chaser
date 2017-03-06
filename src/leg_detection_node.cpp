#include <ros/ros.h>
#include <leg_chaser/leg_detection.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leg_detection_node");

	Legdet legdet;
	legdet.SetClusterThreshold(0.03);
	legdet.SetNanSkipNum(3);
	legdet.SetLidarError(0.03);
	legdet.SetFittingMinPoints(30);
	legdet.SetLegDRange(0.08, 0.16);
	legdet.SetErrorBarThreshold(0.004),
	legdet.Proccessing();
	
	return 0;
}

#include <ros/ros.h>
#include <leg_chaser/leg_detection.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leg_detection_node");

	Legdet legdet;
	legdet.SetClusterThreshold(0.10);
	legdet.SetFittingMinPoints(30);
	legdet.SetLegDRange(0.08, 0.13);
	legdet.Proccessing();
	
	return 0;
}

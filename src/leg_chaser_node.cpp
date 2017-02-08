#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <leg_chaser/leg_detection.h>

Legdet legdet;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	legdet.scanwrite(*msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leg_detection_node");
	ros::NodeHandle node_("~");
	ros::Rate looprate(30);
	ros::Subscriber scansub = node_.subscribe("/scan", 1, &scanCallback);

	legdet.set_label_value(0.04);
	legdet.set_anb_value(3);

	while(ros::ok())
	{
		legdet.labeling();
		legdet.label_disp();
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}

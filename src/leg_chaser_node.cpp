#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <leg_chaser/leg_detection.h>

using namespace std;
sensor_msgs::LaserScan scanresult;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	scanresult = *msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leg_detection_node");
	ros::NodeHandle node_("~");
	ros::Rate looprate(30);
	ros::Subscriber scansub = node_.subscribe("/scan", 1, &scanCallback);

	Legdet legdet;

	while(ros::ok())
	{
		legdet.detection(scanresult);
		ros::spinOnce();
		looprate.sleep();
	}

	return 0;
}

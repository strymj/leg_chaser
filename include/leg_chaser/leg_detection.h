#ifndef LEG_DETECTION_H_
#define LEG_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class Legdet{
	public:
		void detection(sensor_msgs::LaserScan&);
	private:
};


#endif

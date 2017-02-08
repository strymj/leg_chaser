#ifndef LEG_DETECTION_H_
#define LEG_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class Legdet{
	public:
		Legdet();
		void scanwrite(sensor_msgs::LaserScan);
		void set_label_value(double);
		void set_anb_value(int val);
		void labeling();
		void label_disp();
	private:
		sensor_msgs::LaserScan scan;
		int rangemax;
		struct Region{
			int begin;
			int end;
			int points;
		};
		int allow_nan_between;
		double label_value;
		vector<Region> region;
		void region_detection();
};


#endif

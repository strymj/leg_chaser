#include <leg_chaser/leg_detection.h>
#include <Eigen/Core>

using namespace std;

void Legdet::detection(sensor_msgs::LaserScan& scan)
{
	if(scan.ranges.size()) {
		int i=0;
		while(i<scan.ranges.size()) {
			double diff = scan.ranges[i]-scan.ranges[i+1];
			cout<<scan.ranges[i]<<" - "<<scan.ranges[i+1]<<" = "<<diff<<endl;
			i++;
		}

	}
}


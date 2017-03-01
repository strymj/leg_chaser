#ifndef LEG_DETECTION_H_
#define LEG_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/core.hpp>

class Legdet{
	public:
		Legdet();
		void SetClusterThreshold(double);
		void SetFittingMinPoints(int);
		void SetLegDRange(double,double);
		void Proccessing();
	

	private:
		ros::NodeHandle node_;
		ros::Rate looprate;
		ros::Subscriber ScanSub;
		ros::Publisher ScanClusteredPub;
		ros::Publisher ScanLegPub;
		sensor_msgs::LaserScan scan;
		sensor_msgs::LaserScan scanClustered;
		std::string ScanTopic;
		std::string PubClusteredTopic;
		std::string PubScanTopic; 
		double ClusterThreshold;
		int FittingMinPoints;
		double LegDMax;
		double LegDMin;

		std::vector<int> I;
		std::vector<double> DistanceList;
		struct Region 
		{
			int begin;
			int end;
		};
		std::vector<Region> ClusterList;
		struct Circle
		{
			int i;
			double x;
			double y;
			double d;
		};
		std::vector<Circle> CircleList;

		void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
		void ScanWrite(sensor_msgs::LaserScan);
		void Clustering();
		void CircleFitting();
		void SetLegIntensities();
		void PublishScan(ros::Publisher&, sensor_msgs::LaserScan&);
		void VectorClear();

		double CalcDist(int,int);
		void LSM(int);
};

#endif

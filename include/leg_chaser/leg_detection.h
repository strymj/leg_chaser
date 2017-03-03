#ifndef LEG_DETECTION_H_
#define LEG_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core.hpp>

class Legdet{
	public:
		Legdet();
		void SetClusterThreshold(double);
		void SetNanSkipNum(int);
		void SetErrorBarThreshold(double);
		void SetLidarError(double);
		void SetFittingMinPoints(int);
		void SetLegDRange(double,double);
		void Proccessing();
	

	private:
		ros::NodeHandle node_;
		ros::Rate looprate;
		bool subscribed;
		ros::Subscriber ScanSub;
		ros::Publisher ClusteredScanPub;
		ros::Publisher LegScanPub;
		ros::Publisher LegPointsPub;
		ros::Publisher PeoplePub;
		sensor_msgs::LaserScan scan;
		sensor_msgs::LaserScan ClusteredScan;
		sensor_msgs::PointCloud LegPoints;
		geometry_msgs::PoseStamped People;
		std::string ScanTopic;
		std::string ClusteredScanTopic;
		std::string LegScanTopic; 
		std::string LegPointsTopic;
		std::string PeopleTopic;
		double ClusterThreshold;
		int NanSkipNum;
		double ErrorBarThreshold;
		double LidarError;
		int FittingMinPoints;
		double LegDMax;
		double LegDMin;

		std::vector<int> I;
		std::vector<double> DistanceList;
		struct Range 
		{
			int Ibgn;
			int Iend;
		};
		struct Circle
		{
			double r;
			double x;
			double y;
			double ErrorBar;
		};
		struct Cluster
		{
			Range range;
			Circle circle;
		};
		std::vector<Cluster> ClusterList;

		void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
		void ScanWrite(sensor_msgs::LaserScan);
		void Clustering();
		void LegDetection();
		void VectorClear();

		bool isFar(int,int);   // scan.ranges number1, number2
		double CalcDist(int,int);   // scan.ranges number1, number2
		void LSM(int);   // ClusterList number
		double CalcErrorBar(int);   // CircleList number
};

#endif

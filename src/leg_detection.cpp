#include <leg_chaser/leg_detection.h>

using namespace std;

Legdet::Legdet():
	node_("~"),
	looprate(30),	
	ScanTopic("/scan"),
	ClusteredScanTopic("ClusteredScan"),
	LegScanTopic("LegScan"),
	LegPointsTopic("LegPoints"),
	PeopleTopic("People"),
	LegDMin(0.08),
	LegDMax(0.16),
	ClusterThreshold(0.03),
	NanSkipNum(4),
	LidarError(0.0),
	ErrorBarThreshold(0.003),
	FittingMinPoints(20)
{
	subscribed = false;
	ScanSub = node_.subscribe(ScanTopic, 1, &Legdet::scanCallback, this);
	ClusteredScanPub = node_.advertise<sensor_msgs::LaserScan>(ClusteredScanTopic, 1);
	LegScanPub = node_.advertise<sensor_msgs::LaserScan>(LegScanTopic, 1);
	LegPointsPub = node_.advertise<sensor_msgs::PointCloud>(LegPointsTopic, 1);
	PeoplePub = node_.advertise<geometry_msgs::PoseStamped>(PeopleTopic, 1);
}

void Legdet::Proccessing()
{
	while(ros::ok())
	{
		if(subscribed) {
			Clustering();
			LegDetection();
			ClusteredScanPub.publish(ClusteredScan);
			LegScanPub.publish(scan);
			LegPointsPub.publish(LegPoints);
			VectorClear();
			subscribed = false;
		}

		cout<<"<roopEnd>"<<endl;

		ros::spinOnce();
		looprate.sleep();
	}
}

void Legdet::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ScanWrite(*msg);
	subscribed = true;
}

void Legdet::ScanWrite(sensor_msgs::LaserScan msg)
{
	scan = msg;
	ClusteredScan = msg;
}

void Legdet::SetClusterThreshold(double value)
{
	if(value<0) {
		cout<<"cannot set Cluster Threshold (>0[m])"<<endl;
	}
	else {
		ClusterThreshold = value;
	}
}

void Legdet::SetErrorBarThreshold(double value)
{
	if(value<0) {
		cout<<"cannot set ErrorBar Threshold (>0[m])"<<endl;
	}
	else {
		ErrorBarThreshold = value;
	}
}

void Legdet::SetNanSkipNum(int value)
{
	if(value<0) {
		cout<<"cannot set Nan Skip Num (>0)"<<endl;
	}
	else {
		NanSkipNum = value;
	}
}

void Legdet::SetLidarError(double value)
{
	if(value<0) {
		cout<<"cannot set Lidar Error (>0[%])"<<endl;
	}
	else {
		LidarError = value;
	}
}

void Legdet::SetFittingMinPoints(int value)
{
	if(value<0) {
		cout<<"cannot set Fitting Min Points (>0[points])"<<endl;
	}
	else {
		FittingMinPoints = value;
	}
}

void Legdet::SetLegDRange(double min, double max)
{
	if(min<0 || max<0) {
		cout<<"cannot set Leg Diameter Range (RangeMin,Max>0[m])"<<endl;
	}

	if(min<max) {
		LegDMin = min;
		LegDMax = max;
	}
	else {
		LegDMin = max;
		LegDMax = min;
	}
}

void Legdet::Clustering()
{
	// make nan removed list
	for(int i=0; i<scan.ranges.size(); ++i)
	{
		scan.intensities.push_back(0);
		ClusteredScan.intensities.push_back(0);
		if(!isnan(scan.ranges[i])) I.push_back(i);
	}

	// make distance list 
	Cluster cluster = { {0,0}, {-1,0,0} };
	int inte = 0;
	for(int i=0; i+1<I.size(); ++i) {
		double dist = CalcDist(I[i], I[i+1]);
		DistanceList.push_back(dist);
		if(isFar(I[i], I[i+1]) || NanSkipNum<=I[i+1]-I[i]) {
			//if(i+2<I.size() && !isFar(I[i], I[i+2])) {
			//	I.erase(I.begin()+i+1);
			//}
			//else {
			cluster.range.Iend = i;
			ClusterList.push_back(cluster);
			for(int j=cluster.range.Ibgn; j<=cluster.range.Iend; ++j) {
				ClusteredScan.intensities[I[j]] = inte;
			}
			inte += 1;
			cluster.range.Ibgn = i+1;
			//}
		}
	}
}

void Legdet::LegDetection()
{
	// Circle Fitting using Least Squares Method
	for(int i=0; i<ClusterList.size(); ++i) {
		double dist = CalcDist(I[ClusterList[i].range.Ibgn], I[ClusterList[i].range.Iend]);
		if(LegDMin<dist && dist<LegDMax) {
			LSM(i);
		}
	}
}

void Legdet::VectorClear()
{
	I.clear();
	DistanceList.clear();
	ClusterList.clear();
	LegPoints.points.clear();
}

bool Legdet::isFar(int a, int b)
{
	bool isFar = false;
	double dist = CalcDist(a, b);
	double threshold = ClusterThreshold + LidarError * (scan.ranges[a] + scan.ranges[b]) /2;
	if(threshold<dist) isFar = true;
	return isFar;
}

double Legdet::CalcDist(int a, int b)
{
	return sqrt(pow(scan.ranges[a],2)
			+pow(scan.ranges[b],2)
			-2*scan.ranges[a]*scan.ranges[b]*cos((b-a)*scan.angle_increment));
}

void Legdet::LSM(int CLN)
{
	double Sig_x2 = 0.0;
	double Sig_xy = 0.0;
	double Sig_y2 = 0.0;
	double Sig_x  = 0.0;
	double Sig_y  = 0.0;
	double Sig_1  = 0.0;
	double Sig_xa = 0.0;
	double Sig_ya = 0.0;
	double Sig_a  = 0.0;

	for(int i=ClusterList[CLN].range.Ibgn; i<=ClusterList[CLN].range.Iend; ++i) {
		double angle = scan.angle_min + scan.angle_increment * I[i];
		double x = scan.ranges[I[i]] * cos(angle);
		double y = scan.ranges[I[i]] * sin(angle);
		double a = x*x + y*y;
		Sig_x2 += x*x;
		Sig_xy += x*y;
		Sig_y2 += y*y;
		Sig_x  += x;
		Sig_y  += y;
		Sig_1  += 1.0;
		Sig_xa += x * a;
		Sig_ya += y * a;
		Sig_a  += a;
	}

	cv::Mat LSMmat = (cv::Mat_<double>(3,3) <<
			Sig_x2, Sig_xy, Sig_x,
			Sig_xy, Sig_y2, Sig_y,
			Sig_x , Sig_y , Sig_1);

	cv::Mat LSMvec = (cv::Mat_<double>(3,1) <<
			-Sig_xa,
			-Sig_ya,
			-Sig_a);

	cv::Vec3d ABC = (cv::Vec3d)cv::Mat1d(LSMmat.inv() * LSMvec);

	double cx = -ABC[0] / 2;
	double cy = -ABC[1] / 2;
	double cr = sqrt( pow(cx, 2) + pow(cy, 2) - ABC[2] );
	ClusterList[CLN].circle.x = cx;
	ClusterList[CLN].circle.y = cy;
	ClusterList[CLN].circle.r = cr;
	ClusterList[CLN].circle.ErrorBar = CalcErrorBar(CLN);

	cout<<"ErrorBar"<<ClusterList[CLN].circle.ErrorBar<<endl;

	for(int i=ClusterList[CLN].range.Ibgn; i<=ClusterList[CLN].range.Iend; ++i) {
		scan.intensities[I[i]] = 1.0;
	}

	LegPoints.header = scan.header;
	if(ClusterList[CLN].circle.ErrorBar<ErrorBarThreshold
			&& LegDMin<2*cr && 2*cr<LegDMax) {
		geometry_msgs::Point32 LegPoint;
		LegPoint.x = cx;
		LegPoint.y = cy;
		LegPoint.z = 0.0;
		LegPoints.points.push_back(LegPoint);
	}
}

double Legdet::CalcErrorBar(int CLN)
{
	double SigError = 0.0;
	for(int i=ClusterList[CLN].range.Ibgn; i<=ClusterList[CLN].range.Iend; ++i) {
		double angle = scan.angle_min + scan.angle_increment * I[i];
		double x = scan.ranges[I[i]] * cos(angle);
		double y = scan.ranges[I[i]] * sin(angle);
		SigError += fabs(ClusterList[CLN].circle.r
				- sqrt(pow(x-ClusterList[CLN].circle.x, 2) + pow(y-ClusterList[CLN].circle.y, 2)));
	}
	return SigError / (ClusterList[CLN].range.Iend-ClusterList[CLN].range.Ibgn+1);
}


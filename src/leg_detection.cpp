#include <leg_chaser/leg_detection.h>
#define ClstBgn(iCir) ClusterList[CircleList[iCir].i].begin
#define ClstEnd(iCir) ClusterList[CircleList[iCir].i].end

using namespace std;

Legdet::Legdet():
	node_("~"),
	looprate(30),	
	ScanTopic("/scan"),
	PubClusteredTopic("clustered"),
	PubScanTopic("leg"),
	ClusterThreshold(0.03),
	FittingMinPoints(20)
{
	ScanSub = node_.subscribe(ScanTopic, 1, &Legdet::scanCallback, this);
	ScanClusteredPub = node_.advertise<sensor_msgs::LaserScan>(PubClusteredTopic, 1);
	ScanLegPub = node_.advertise<sensor_msgs::LaserScan>(PubScanTopic, 1);
}

void Legdet::Proccessing()
{
	while(ros::ok())
	{
		Clustering();
		CircleFitting();
		SetLegIntensities();
		PublishScan(ScanClusteredPub, scanClustered);
		PublishScan(ScanLegPub, scan);
		VectorClear();

		cout<<"<roopEnd>"<<endl;

		ros::spinOnce();
		looprate.sleep();
	}
}

void Legdet::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	ScanWrite(*msg);
}

void Legdet::ScanWrite(sensor_msgs::LaserScan msg)
{
	scan = msg;
	scanClustered = msg;
}

void Legdet::SetClusterThreshold(double value)
{
	if(value<0) {
		cout<<"cannot set Cluster Threshold (>0[m])"<<endl;
	}else {
		ClusterThreshold = value;
	}
}

void Legdet::SetFittingMinPoints(int value)
{
	if(value<0) {
		cout<<"cannot set Fitting Min Points (>0[points])"<<endl;
	}else {
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
		scanClustered.intensities.push_back(0);
		if(!isnan(scan.ranges[i])) I.push_back(i);
	}

	// make distance list 
	Region region;
	region.begin = 0;
	double inte = 0.0;
	for(int i=0; i+1<I.size(); ++i) {
		double dist = CalcDist(I[i], I[i+1]);
		DistanceList.push_back(dist);
		if(ClusterThreshold<dist) {
			if(i+2<I.size() && CalcDist(I[i], I[i+2])<ClusterThreshold) {
				I.erase(I.begin()+i+1);
			}
			else {
				region.end = i;
				for(int j=region.begin; j<=region.end; ++j) {
					scanClustered.intensities[I[j]] = inte;
				}
				inte += 10.0;
				ClusterList.push_back(region);
				region.begin = i+1;
			}
		}
	}
}

void Legdet::CircleFitting()
{
	for(int i=0; i<ClusterList.size(); ++i) {
		if(FittingMinPoints < ClusterList[i].end-ClusterList[i].begin+1) {
			LSM(i);
		}
	}
}

void Legdet::SetLegIntensities()
{
	for(int iCir=0; iCir<CircleList.size(); ++iCir) {
		if(LegDMin<CircleList[iCir].d && CircleList[iCir].d<LegDMax) {
			cout<<"ErrorBar"<<CalcErrorBar(iCir)<<endl;
			for(int j=ClstBgn(iCir); j<=ClstEnd(iCir); ++j) {
				scan.intensities[I[j]] = 1.0;
			}
			//cout<<"intensitiy set."<<endl;
		}
	}
}

void Legdet::PublishScan(ros::Publisher& pub, sensor_msgs::LaserScan& msg)
{
	pub.publish(msg);
}

void Legdet::VectorClear()
{
	I.clear();
	DistanceList.clear();
	ClusterList.clear();
	CircleList.clear();
}

double Legdet::CalcDist(int a, int b)
{
	return sqrt(pow(scan.ranges[a],2)
			+pow(scan.ranges[b],2)
			-2*scan.ranges[a]*scan.ranges[b]*cos((b-a)*scan.angle_increment));
}

void Legdet::LSM(int Cli)
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

	for(int i=ClusterList[Cli].begin; i<=ClusterList[Cli].end; ++i) {
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

	Circle circle;
	circle.i = Cli;
	circle.x = - ABC[0] / 2;
	circle.y = - ABC[1] / 2; 
	circle.d = 2 * sqrt( pow(circle.x, 2) + pow(circle.y, 2) - ABC[2] );
	CircleList.push_back(circle);
	cout<<"x,y,d =  "<<circle.x<<" , "<<circle.y<<" , "<<circle.d<<endl;
}

double Legdet::CalcErrorBar(int iCir)
{
	double SigError2 = 0.0;
	for(int i=ClstBgn(iCir); i<=ClstEnd(iCir); ++i) {
		double angle = scan.angle_min + scan.angle_increment * I[i];
		double x = scan.ranges[I[i]] * cos(angle);
		double y = scan.ranges[I[i]] * sin(angle);
		SigError2 += pow(x-CircleList[iCir].x, 2) + pow(y-CircleList[iCir].y, 2);
	}
	return SigError2 / (ClstEnd(iCir)-ClstBgn(iCir)+1);
}



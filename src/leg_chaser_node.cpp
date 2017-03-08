#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

geometry_msgs::PoseStamped people;
bool subscribed = false;

void PeoplePoseCallback(const geometry_msgs::PoseStamped& msg)
{
	people = msg;
	subscribed = true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "leg_chaser_node");
	ros::NodeHandle node_("~");

	std::string PeoplePoseTopic_, MovecmdTopic_;
	double LinearGain_, AngularGain_, KeepDist_, MaxDist_;
	node_.param("PeoplePoseTopic", PeoplePoseTopic_, std::string("/leg_detection/People"));
	node_.param("MovecmdTopic", MovecmdTopic_, std::string("/omni_movecmd"));
	node_.param("LinearGain", LinearGain_, 2.0);
	node_.param("AngluarGain", AngularGain_, 2.3);
	node_.param("KeepDistance", KeepDist_, 1.2);
	node_.param("MaxDistance", MaxDist_, 2.0);

	ros::Subscriber PeoplePoseSub = node_.subscribe(PeoplePoseTopic_, 1, PeoplePoseCallback);
	ros::Publisher MovecmdPub = node_.advertise<std_msgs::Float32MultiArray>(MovecmdTopic_, 1);
	ros::Rate looprate(30);

	while(ros::ok()) {
		if(subscribed) {
			double distance = sqrt(pow(people.pose.position.x, 2) + pow(people.pose.position.y, 2));
			double linear = 0.0;
			double angular = 0.0;
			if(distance<MaxDist_) {
				linear = LinearGain_ * (distance - KeepDist_);
				if(linear<0) linear = 0;
				angular = AngularGain_ * atan2(people.pose.position.y, people.pose.position.x);
			}

			std_msgs::Float32MultiArray movecmd;
			movecmd.data.push_back(linear);
			movecmd.data.push_back(0.0);
			movecmd.data.push_back(angular);
			MovecmdPub.publish(movecmd);
		}

		subscribed = false;
		ros::spinOnce();
		looprate.sleep();
	}


	return 0;
}

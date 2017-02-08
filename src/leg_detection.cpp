#include <leg_chaser/leg_detection.h>
#include <Eigen/Core>

Legdet::Legdet(){
	label_value = 0.03;
	allow_nan_between = 1;
}

void Legdet::scanwrite(sensor_msgs::LaserScan msg)
{
	scan = msg;
	rangemax = scan.ranges.size();
}

void Legdet::region_detection()
{
	static Region re;
	static int nan_between;
	static int i;
	i = 0;

	while(i<rangemax) {

		nan_between = 0;
		re.begin = -1;
		re.end   = -1;
		re.points = 0;

		// region begin detect
		while(i<rangemax)
		{
			//cout<<"begin_detection"<<endl;
			if(!isnan(scan.ranges[i])) {
				re.begin = i;
				i++;
				break;
			}
			i++;
		}

		// region end detect
		while(1)
		{
			if(i>rangemax) {
				re.end = rangemax;
				break;
			}
			else if(isnan(scan.ranges[i])) {
				nan_between++;
				i++;
				re.points--;
			}
			else { 
				if(nan_between>allow_nan_between || fabs(scan.ranges[i]-scan.ranges[i-nan_between-1])>label_value) {
					i--;
					re.end = i;
					break;
				}
				else {
					nan_between = 0;
					i++;
				}
			}
		}
		
		// next region
		re.points += re.end - re.begin + 1;
		region.push_back(re);
		i++;
	
	}
}

void Legdet::set_label_value(double val)
{
	if(val<0) {
		cout<<"cannot set label_value (0<value)"<<endl;
	}else {
		label_value = val;
	}
}

void Legdet::set_anb_value(int val)
{
	if(val<1) {
		cout<<"cannot set allow_nan_between (1<=value)"<<endl;
	}else {
		allow_nan_between = val;
	}
}

void Legdet::labeling()
{
	if(scan.ranges.size()) {
		region.clear();
		region_detection();
	}

}

void Legdet::label_disp()
{
	cout<<"region_num = "<<region.size()<<endl;
	for(int i=0; i<region.size(); i++) {
		cout<<"region["<<i<<"] = "<<region[i].begin<<"-"<<region[i].end<<" ("<<region[i].points<<")"<<endl;
	}
	cout<<endl;
}

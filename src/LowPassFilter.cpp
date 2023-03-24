#include "ros/ros.h"
#include "classwork3/sin_msg.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include "boost/thread.hpp"

using namespace std;

class SIN_ELAB {
	public:
		SIN_ELAB();
		void filter();
		float getValue() {return _currValue;}
		void startFilter();
		void saveData(classwork3::sin_msg::ConstPtr msg);
	private:
		float _currValue;
		float _input;
		float _K; //filter gain
		float _T; //filter time constant
		float _Ts; //filter time sample
		ros::NodeHandle _nh;
		ros::Subscriber _topic_sub;
		ros::Publisher _topic_pub;
		ros::Rate _rate;
};

SIN_ELAB::SIN_ELAB(): _rate(10) {
	_currValue = 0;
	_input = 0;
	_K = 1;
	_T = 1;
	_Ts = 0.1;
	_topic_sub = _nh.subscribe("/sin_wave", 1, &SIN_ELAB::saveData, this);
	_topic_pub = _nh.advertise<std_msgs::Float32>("/filtered_numbers", 1);
}

//Callback function: the input of the function is the data to read
void SIN_ELAB::saveData(classwork3::sin_msg::ConstPtr msg) {
	ROS_INFO("I heard: amplitude = [%f], period = [%f], value = [%f]", msg->amplitude, msg->period, msg->value);
	_input = msg->value;
}

void SIN_ELAB::filter() {
	std_msgs::Float32 f;
	while (true) {
		_currValue = (1-_Ts/_T)*_currValue + _K*_Ts/_T*_input;
		ROS_INFO("Filtered value: %f", _currValue);
		f.data = _currValue;
		_topic_pub.publish(f);
		_rate.sleep();
	}
	
}

void SIN_ELAB::startFilter() {
	boost::thread(&SIN_ELAB::filter, this);
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "LowPassFilter");

	//Create the ROS_SUB class object
	SIN_ELAB elab;

	elab.startFilter();
	
	/*while(ros::ok()) {
		elab.saveData();
		rate.sleep();
	}*/

	ros::spin();

	return 0;
}

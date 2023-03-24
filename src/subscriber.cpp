#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;

//Use a class to store the topic data 
//	Just a choice: the best way to share data between a main loop and the ROS callbacks
class ROS_SUB {
	public:
		ROS_SUB();
		void topic_cb( std_msgs::Float32ConstPtr data);
	
	private:
		ros::NodeHandle _nh;
		//Subscriber object
		ros::Subscriber _topic_sub;
};

ROS_SUB::ROS_SUB() {
	//Initialize a subscriber:
	//	Input: 	topic name: /numbers
	//			queue:	1
	//			Callback function
	//			Object context: the value of data members
	_topic_sub = _nh.subscribe("/filtered_numbers", 1, &ROS_SUB::topic_cb, this);
}

//Callback function: the input of the function is the data to read
//	In this function, a smart pointer is used
void ROS_SUB::topic_cb( std_msgs::Float32ConstPtr data) {

	//data is a pointer of std_msgs::Float32 type
	//	to access to its fiel, the "." can not be used
	ROS_INFO("Listener: %f", data->data);
}

int main( int argc, char** argv ) {

	//Init the ros node with ros_subscriber name
	ros::init(argc, argv, "ros_subscriber");

	//Create the ROS_SUB class object
	ROS_SUB rs;
	
	//ros::spin() blocks the main thread from exiting until ROS invokes a shutdown - via a Ctrl + C for example
	// It is written as the last line of code of the main thread of the program.
	//Also the spin invokes the callbacks to flush their queue and process incoming data
	ros::spin(); 

	//----This function will be never overcome

	return 0;
}



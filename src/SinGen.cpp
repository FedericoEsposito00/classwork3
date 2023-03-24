#include "ros/ros.h" 
#include "classwork3/sin_msg.h"
#include <cmath>

using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "SinGen");

    ros::NodeHandle n;
    ros::Publisher p = n.advertise<classwork3::sin_msg>("/sin_wave", 1);

    float amplitude = 1; // default sin parameters
    float period = 2*M_PI;

    if (argc>=2) {
        amplitude = atof(argv[1]);
    }
    if (argc>=3) {
        period = atof(argv[2]);
    }

    classwork3::sin_msg s;

    s.amplitude = amplitude; 
    s.period = period;
    
    double secs;

    ros::Rate rate(10);

    while(ros::ok()) {
        secs = ros::Time::now().toSec();
        s.value = amplitude*sin(2*M_PI*secs/period);
        ROS_INFO("%f",s.value); 
        p.publish(s);
        rate.sleep();
    }

    return 0;
}

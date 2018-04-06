#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mavros_msgs/AttitudeTarget.h"

float currPos[3];
float currVel[3];
float desPos[3];



void setposCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	desPos[0] = msg->pose.position.x;
	desPos[1] = msg->pose.position.y;
}

void currposCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	currPos[0] = msg->pose.pose.position.x;
	currPos[1] = msg->pose.pose.position.y;
}

void currvelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
	currVel[0] = msg->twist.linear.x;
	currVel[1] = msg->twist.linear.y;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "position_controller");

	ros::NodeHandle n;

	ros::Subscriber pos_set_sub = n.subscribe("setpos", 100, setposCallback);
	ros::Subscriber curr_pos_sub = n.subscribe("currpos", 100, currposCallback);
	ros::Subscriber curr_vel_sub = n.subscribe("currvel", 100, currvelCallback);
	ros::Publisher att_pub = n.advertise<mavros_msgs::AttitudeTarget>("setatt", 100);

	ros::Rate loop_rate(50);

	while (ros::ok()) {


		ros::spin();
		loop_rate.sleep();
	}

	return 0;

}
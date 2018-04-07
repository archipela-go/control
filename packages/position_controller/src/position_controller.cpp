#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <stdlib.h>
#include <math.h>

float currPos[3];
float currVel[3]; //xdot, ydot, zdot
float desPos[3];
float oldDesPos[3];
double currEuler[3]; //rol, pitch, yaw
double desEuler[3]; //roll, ptich, yaw
int firstTime; 



void setposCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	if (firstTime) {
		desPos[0] = msg->pose.position.x;
		desPos[1] = msg->pose.position.y;
		oldDesPos[0] = desPos[0];
		oldDesPos[1] = desPos[1];
		firstTime = 0;
	}
	oldDesPos[0] = desPos[0];
	oldDesPos[1] = desPos[1];
	desPos[0] = msg->pose.position.x;
	desPos[1] = msg->pose.position.y;
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.orientation, q);
	tf::Matrix3x3(q).getRPY(desEuler[0], desEuler[1], desEuler[2]);	
	desEuler[2] = angles::normalize_angle_positive(desEuler[2]);
}

void currposCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	currPos[0] = msg->pose.pose.position.x;
	currPos[1] = msg->pose.pose.position.y;
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
	tf::Matrix3x3(q).getRPY(currEuler[0], currEuler[1], currEuler[2]);	
	currEuler[2] = angles::normalize_angle_positive(currEuler[2]);
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

	firstTime = 1;

	while (ros::ok()) {
		//calculate speed
		double U = sqrt(pow(currVel[0], 2) + pow(currVel[1], 2));

		//calculate angle of velocity vector(CHECK)
		double vel_angle = atan2(currVel[1], currVel[0]);

		double alpha = atan2((desPos[1] - oldDesPos[1]), (desPos[0] - oldDesPos[0]));

		double s = (currPos[0] - oldDesPos[0])*cos(alpha) + (currPos[1] - oldDesPos[1])*sin(alpha);
		double e = -(currPos[0] - oldDesPos[0])*sin(alpha) + (currPos[1] - oldDesPos[1])*cos(alpha);

		//page 269

		ros::spin();
		loop_rate.sleep();
	}

	return 0;

}
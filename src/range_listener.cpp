/*
 * Author: Anis Koubaa for Gaitech EDU
 * Year: 2016
 *
 */

#include "ros/ros.h"
#include <sensor_msgs/Range.h>
#include "geometry_msgs/Twist.h"
#include <signal.h>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber range_subscriber;   

const double PI = 3.14159265359;
float range_dist = 10;


void RangeCallback(const sensor_msgs::Range::ConstPtr& msg);
void moveforward(double speed);
void myrotate(double angularspeed);
void mySigintHandler(int sig);

void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);
void bouncee();

int main(int argc, char **argv)
{
    // Initiate a new ROS node named "listener"
	ros::init(argc, argv, "range_listener_node");
	//create a node handle: it is reference assigned to a new node
	ros::NodeHandle node;
    double speed, angular_speed;
	double distance, angle;
	bool isForward, clockwise;

    velocity_publisher = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    range_subscriber = node.subscribe("ultrasound_range", 1000, RangeCallback);

	ros::Rate loop_rate(10); 
    while (ros::ok()) {   

		if(range_dist<25){
			moveforward(-.5);
			ros::Duration(0.7).sleep(); //sleep .5 seconds

			myrotate(15);
			ros::Duration(0.5).sleep(); 
		}
		else{
			moveforward(.5);
		}
        

		signal(SIGINT, mySigintHandler);
		
        ros::spinOnce();
		loop_rate.sleep();
    }
    
	moveforward(0);

    ros::spin();
    return 0;
}


// Topic messages callback
void RangeCallback(const sensor_msgs::Range::ConstPtr& range_msg)
{
    ROS_INFO("Sonar Seq: [%d]", range_msg->header.seq);
    ROS_INFO("Sonar Range: [%f]", range_msg->range);
    range_dist = range_msg->range;


}

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, bool isForward){
	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	if (isForward)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
		//cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}


void rotate (double angular_speed, double relative_angle, bool clockwise){

	geometry_msgs::Twist vel_msg;
	//set a random linear velocity in the x-axis
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clockwise)
		vel_msg.angular.z =-abs(angular_speed);
	else
		vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<relative_angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

void moveforward(double speed){
	ROS_INFO("move forward");
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = speed;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);
}

void myrotate(double angularspeed){
	ROS_INFO("rotate");
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =	angularspeed;
	velocity_publisher.publish(vel_msg); 
}

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  moveforward(0);
  ros::shutdown();
}

void bouncee(){
    

}
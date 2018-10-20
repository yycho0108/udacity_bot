#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void reset_pose(ros::NodeHandle& nh){
	auto msg_in = ros::topic::waitForMessage<nav_msgs::Odometry>("/ground_truth_pose");//, nh, 0.5);
	geometry_msgs::PoseWithCovarianceStamped msg_out;
	msg_out.header = msg_in->header;
	msg_out.pose = msg_in->pose;

	ros::Publisher p0_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

	ros::Rate rate(50.0);
	while(ros::ok()){
		if(p0_pub.getNumSubscribers() > 0){
			p0_pub.publish(msg_out);
			break;
		}
		ros::spinOnce();
		rate.sleep();
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_goal");
	ros::NodeHandle nh("~");

	double gx,gy,gh;
	bool reset;
	nh.param<double>("gx", gx, 0.995);
	nh.param<double>("gy", gy, -2.99);
	nh.param<double>("gh", gh, 0.0);

	nh.param<bool>("reset", reset, false);
	if(reset){reset_pose(nh);}

	ROS_INFO("Goal : (%.3f,%.3f,%.3f)", gx, gy, gh);

    // Spin a thread
    MoveBaseClient ac("move_base", true);

    // Wait for the action server to come up
    ROS_INFO("Waiting for the move_base action server");
    ac.waitForServer(ros::Duration(5));

    ROS_INFO("Connected to move_base server");

    move_base_msgs::MoveBaseGoal goal;

    // Send goal pose
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Do NOT modify the following for final submission.
    goal.target_pose.pose.position.x = gx;
    goal.target_pose.pose.position.y = gy;

    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = sin(gh/2.);
    goal.target_pose.pose.orientation.w = cos(gh/2.);

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Excellent! Your robot has reached the goal position.");
    else
        ROS_INFO("The robot failed to reach the goal position");

    return 0;
}

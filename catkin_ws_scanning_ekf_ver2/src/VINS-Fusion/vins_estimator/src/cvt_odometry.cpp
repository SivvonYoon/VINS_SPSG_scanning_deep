#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher odom_pub;

void callbackFun(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
    nav_msgs::Odometry new_odom;
    new_odom.header = pose_msg->header;
    new_odom.pose   = pose_msg->pose;    
    odom_pub.publish(new_odom);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "convert_odom");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/robot_pose_ekf/odom_combined", 1, callbackFun);
    odom_pub = nh.advertise<nav_msgs::Odometry>("refined_odom", 1000);
    ros::spin();
    return 0;
}



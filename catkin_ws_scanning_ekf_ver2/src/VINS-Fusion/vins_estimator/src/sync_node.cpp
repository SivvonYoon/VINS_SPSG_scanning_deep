#include <vector>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Imu> syncPolicy;

std::string cam_frame, imu_frame;
double cov_imu_rotX,cov_imu_rotY,cov_imu_rotZ,
       cov_odom_transX,cov_odom_transY,cov_odom_transZ,
       cov_odom_rotX,cov_odom_rotY,cov_odom_rotZ;
int sync_redundancy;

ros::Publisher imu_pub;
ros::Publisher vo_pub;

void sync_Callback(const nav_msgs::Odometry::ConstPtr& vo_msg,
                   const sensor_msgs::Imu::ConstPtr& imu_msg)
{
    sensor_msgs::Imu new_imu_msg;
    nav_msgs::Odometry new_odom_msg;
    new_imu_msg.header                 = imu_msg->header;    
    new_imu_msg.header.stamp           = vo_msg->header.stamp;
    new_imu_msg.header.frame_id        = imu_frame;
    new_imu_msg.orientation            = imu_msg->orientation;
    new_imu_msg.orientation_covariance = { cov_imu_rotX, 	0,      0,	 
    					       	           0,  cov_imu_rotY,    	0,  
                        		       	   0,    		0, cov_imu_rotZ};
                        		       
    new_imu_msg.linear_acceleration            = imu_msg->linear_acceleration;
    new_imu_msg.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;
    new_imu_msg.angular_velocity               = imu_msg->angular_velocity;
    new_imu_msg.angular_velocity_covariance    = imu_msg->angular_velocity_covariance;
    new_odom_msg.header          = imu_msg->header;
    new_odom_msg.header.stamp    = new_imu_msg.header.stamp;
    new_odom_msg.header.frame_id = cam_frame;
    new_odom_msg.pose   	  = vo_msg->pose;
    new_odom_msg.pose.covariance = {cov_odom_transX, 0, 0, 0, 0, 0,  
                        	     0, cov_odom_transY, 0, 0, 0, 0,     
                        	     0, 0, cov_odom_transZ, 0, 0, 0,     
                        	     0, 0, 0, cov_odom_rotX, 0, 0,       
                        	     0, 0, 0, 0, cov_odom_rotY, 0,       
                        	     0, 0, 0, 0, 0, cov_odom_rotZ};      
    new_odom_msg.twist = vo_msg->twist;
        
    imu_pub.publish(new_imu_msg);
    vo_pub.publish(new_odom_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "topic_syncer");
    ros::NodeHandle nh;  
    
    // get node params
    nh.getParam("imu_vo/cam_frame", cam_frame);
    nh.getParam("imu_vo/imu_frame", imu_frame);
    nh.getParam("imu_vo/sync_redundancy", sync_redundancy);
    
    // get imu params
    nh.getParam("imu_vo/cov_imu_rotX", cov_imu_rotX);
    nh.getParam("imu_vo/cov_imu_rotY", cov_imu_rotY);
    nh.getParam("imu_vo/cov_imu_rotZ", cov_imu_rotZ);
 
    // get cov params
    nh.getParam("imu_vo/cov_odom_transX", cov_odom_transX);
    nh.getParam("imu_vo/cov_odom_transY", cov_odom_transY);
    nh.getParam("imu_vo/cov_odom_transZ", cov_odom_transZ);
    nh.getParam("imu_vo/cov_odom_rotX", cov_odom_rotX);
    nh.getParam("imu_vo/cov_odom_rotY", cov_odom_rotY);
    nh.getParam("imu_vo/cov_odom_rotZ", cov_odom_rotZ);

    message_filters::Subscriber<sensor_msgs::Imu>   imu_sub(nh,"/imu_data", 1);
    message_filters::Subscriber<nav_msgs::Odometry> vo_sub(nh, "/vins_estimator/odometry", 1);
    message_filters::Synchronizer<syncPolicy>       sync(syncPolicy(sync_redundancy), vo_sub, imu_sub);
    sync.registerCallback(boost::bind(&sync_Callback, _1, _2));

    imu_pub = nh.advertise<sensor_msgs::Imu>("/sync_imu", 1);
    vo_pub  = nh.advertise<nav_msgs::Odometry>("/sync_odom", 1);
    
    ros::spin();
    return 0;
}





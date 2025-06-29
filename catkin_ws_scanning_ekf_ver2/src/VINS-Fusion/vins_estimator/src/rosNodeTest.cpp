/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
queue<nav_msgs::OdometryPtr> refined_Odom_buf;
std::mutex m_buf;

int left_count = 0;
int right_count = 0;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

void refined_odom_callback(const nav_msgs::OdometryPtr &odom_msg)
{
    m_buf.lock();
    refined_Odom_buf.push(odom_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            int seq0, seq1;
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                seq0 = img0_buf.front()->header.seq;
                seq1 = img1_buf.front()->header.seq;
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();

                // 추가
                int t0 = static_cast<int>(time0);
                int t1 = static_cast<int>(time1);
                std::string filename0, filename1;
                filename0 = "/home/sivvon/Desktop/Junseok_backup_0410/knu/frames/interbulgo_cam0/seq_" + std::to_string(seq0) + "_time_" + std::to_string(t0) + ".png";
                filename1 = "/home/sivvon/Desktop/Junseok_backup_0410/knu/frames/interbulgo_cam1/seq_" + std::to_string(seq1) + "_time_" + std::to_string(t1) + ".png";

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    image1 = getImageFromMsg(img1_buf.front());
                    cv::imwrite(filename0, image0);
                    cv::imwrite(filename1, image1);
                    img0_buf.pop();
                    img1_buf.pop();

                    if (!refined_Odom_buf.empty())
                    {                        
                        double timeOdom = refined_Odom_buf.front()->header.stamp.toSec();
                        
                        if((time0-timeOdom) < 0.1)
                        {
                            Quaterniond tmp_Q;
                            tmp_Q.x() = refined_Odom_buf.front()->pose.pose.orientation.x;
                            tmp_Q.y() = refined_Odom_buf.front()->pose.pose.orientation.y;
                            tmp_Q.z() = refined_Odom_buf.front()->pose.pose.orientation.z;
                            tmp_Q.w() = refined_Odom_buf.front()->pose.pose.orientation.w;
                            estimator.Rs[WINDOW_SIZE] = tmp_Q.normalized().toRotationMatrix();
                        }
                        refined_Odom_buf.pop();
                    }                    
                }
            }
            m_buf.unlock();

            if(!image0.empty())
            {
                estimator.inputImage(time, image0, image1, seq0);
            }
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    printf("what is this..?\n");

    string config_file;        
    n.getParam("config_file", config_file);    
    readParameters_crvl(config_file);
    estimator.setParameter();

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu;
    if(USE_IMU)
    {
        sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    }
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1;
    ros::Subscriber sub_refinedOdom = n.subscribe("/refined_odom", 100, refined_odom_callback);;

    if(STEREO)
    {
        sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    }
    
    // 봐야할 부분
    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}

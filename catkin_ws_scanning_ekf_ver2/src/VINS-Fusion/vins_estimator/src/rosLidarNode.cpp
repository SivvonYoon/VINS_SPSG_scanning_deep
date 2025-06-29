#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <signal.h>
#include "cmath"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "utility/visualization.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "laser_geometry/laser_geometry.h"
#include "nav_msgs/Path.h"
#include "pcl/io/ply_io.h"
#include <fstream>

queue<nav_msgs::OdometryPtr> odom_buf;
queue<sensor_msgs::LaserScanPtr> laser_buf;

std::vector<nav_msgs::Odometry> odom_data_vector;

nav_msgs::Odometry odom_data;
nav_msgs::Path     path_data;

std::mutex m_buf;

std::string VIO_LIDAR_3D_MAP_PATH;

std::string output_path;

int publishHz = 1;
int publishHzCount = 0;
ros::Publisher pub_lidar;
ros::Publisher pub_path;
nav_msgs::Path path;
sensor_msgs::PointCloud mParkingData;
sensor_msgs::PointCloud mParkingDataOnlyOneQueue;
sensor_msgs::PointCloud2 mParkingData2;

static laser_geometry::LaserProjection projector;

static Eigen::Matrix3d mRLidarLeft2Right;
static Eigen::Matrix3d mRLidar2Cam;
static Eigen::MatrixXd mtLidar2Cam(3,1);

int USE_IMU = 0;



struct QuaternionCV {
    double w, x, y, z;
};

struct EulerAnglesCV {
    double roll, pitch, yaw;
};

void readParameters(std::string config_file); // modify_crvl

void convert_rotatoin_nav_msgs_Odometry(const nav_msgs::OdometryPtr &src, nav_msgs::Odometry &dst)
{
    dst.header = src->header;
    Eigen::Quaterniond mConvertQ(0.7071068, 0, 0, 0.7071068); // x y z w
    
    Eigen::Quaterniond mOriginQ;    
    mOriginQ.x() = src->pose.pose.orientation.x;
    mOriginQ.y() = src->pose.pose.orientation.y;
    mOriginQ.z() = src->pose.pose.orientation.z;
    mOriginQ.w() = src->pose.pose.orientation.w;

    Eigen::Quaterniond mOrigint;
    mOrigint.w() = 0;
    mOrigint.vec().x() = src->pose.pose.position.x;
    mOrigint.vec().y() = src->pose.pose.position.y;
    mOrigint.vec().z() = src->pose.pose.position.z;
    
    Eigen::Quaterniond mTargetQ = mConvertQ * mOriginQ;
    Eigen::Quaterniond mTargett = mConvertQ * mOrigint;
    
    dst.pose.pose.orientation.x;
    dst.pose.pose.orientation.y;
    dst.pose.pose.orientation.z;
    dst.pose.pose.orientation.w;

    dst.pose.pose.position.x = mTargett.vec().x();
    dst.pose.pose.position.y = mTargett.vec().y();
    dst.pose.pose.position.z = mTargett.vec().z();
}

void odom_callback(const nav_msgs::OdometryPtr &odom_msg)
{    
    m_buf.lock();
    //std::cerr << "odom[Pose,Orientation]: [" << odom_msg->header.stamp << "] " << odom_msg->pose.pose.position << ", " << odom_msg->pose.pose.orientation << std::endl;
    //std::cerr << "odom[Pose,Orientation]: [" << odom_msg->header.stamp << "] " << std::endl;    
    // convert_rotatoin_nav_msgs_Odometry(odom_msg, odom_data);

    // geometry_msgs::PoseStamped currPose;
    // currPose.pose = odom_data.pose.pose;
   
    // path_data.poses.push_back(currPose);

    // path_data.header = odom_data.header;
    
    // pub_odom.publish(path_data);
    //odom_buf.push(odom_data);

    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header = odom_msg->header;
    // pose_stamped.header.frame_id = "world";
    // pose_stamped.pose = odom_msg->pose.pose;
    // path.header = odom_msg->header;
    // path.header.frame_id = "world";
    // path.poses.push_back(pose_stamped);
    // pub_path.publish(path);
    
    odom_buf.push(odom_msg);
    odom_data_vector.push_back(*odom_msg);
    m_buf.unlock();
};

void laser_callback(const sensor_msgs::LaserScanPtr& laser_msg)
{
    m_buf.lock();
    //odom_data

    laser_buf.push(laser_msg);
    //std::cerr << "LidarTime: " << laser_msg->header.stamp << std::endl;    
    m_buf.unlock();
};

void update_scanData(const nav_msgs::OdometryPtr &odom_msg, const sensor_msgs::LaserScanPtr& laser_msg, sensor_msgs::PointCloud &_ParkingData, sensor_msgs::PointCloud &_ParkingDataOnlyOneQueue)
{
    sensor_msgs::PointCloud cloud;
    projector.projectLaser(*laser_msg, cloud);

    Eigen::Matrix3d mR;
    Eigen::Quaterniond mQ;
    mQ.x() = odom_msg->pose.pose.orientation.x;
    mQ.y() = odom_msg->pose.pose.orientation.y;
    mQ.z() = odom_msg->pose.pose.orientation.z;
    mQ.w() = odom_msg->pose.pose.orientation.w;
    mR = mQ.normalized().toRotationMatrix();

    Eigen::MatrixXd mt(3,1);
    mt(0,0) =  odom_msg->pose.pose.position.x;
    mt(1,0) =  odom_msg->pose.pose.position.y;
    mt(2,0) =  odom_msg->pose.pose.position.z;
    // std::cerr << "mt: " << mt << std::endl;;
    

    Eigen::MatrixXd mData(3,1);
    Eigen::MatrixXd mConvertData(3,1);
    int preve_dataSize = _ParkingData.points.size();
    _ParkingData.points.resize(preve_dataSize + cloud.points.size());    
    _ParkingDataOnlyOneQueue.points.resize(cloud.points.size());    
    for(int i = 0; i < cloud.points.size(); ++i)
    {
        int j = i + preve_dataSize;
        mData(0,0) = cloud.points[i].x;
        mData(1,0) = cloud.points[i].y;
        mData(2,0) = cloud.points[i].z;

        mConvertData = mR * (mRLidar2Cam * mRLidarLeft2Right * mData + mtLidar2Cam) + mt;
        
        _ParkingData.points[j].x = mConvertData(0,0);
        _ParkingData.points[j].y = mConvertData(1,0);
        _ParkingData.points[j].z = mConvertData(2,0);

        _ParkingDataOnlyOneQueue.points[i].x = mConvertData(0,0);
        _ParkingDataOnlyOneQueue.points[i].y = mConvertData(1,0);
        _ParkingDataOnlyOneQueue.points[i].z = mConvertData(2,0);
    }
    
    _ParkingData.header = odom_msg->header;
    _ParkingDataOnlyOneQueue.header = odom_msg->header;
    
    //std::cerr << "cloud.points.size()" << cloud.points.size() << std::endl;
    //_ParkingData.

    //projector.projectLaser(*laser_msg, _ParkingData);
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        m_buf.lock();
        if (!odom_buf.empty() && !laser_buf.empty())
        {
            double time0 = odom_buf.front()->header.stamp.toSec();
            double time1 = laser_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.05)
            {
                odom_buf.pop();                
                //std::cerr << "[Fusion odom_laser] odom < laser : " << (time0-time1) << std::endl;
            }
            else if(time0 > time1 + 0.05)
            {
                laser_buf.pop();                
                //std::cerr << "[Fusion odom_laser] odom > laser : " << (time0-time1) << std::endl;
            }
            else
            {
                //std::cerr << "[time] odom - lidar: " << time0 - time1 << std::endl;
                update_scanData(odom_buf.front(), laser_buf.front(), mParkingData, mParkingDataOnlyOneQueue);
                
                if(publishHzCount == publishHz)
                {
                    //convertPointCloudToPointCloud2(mParkingData,mParkingData2);
                    convertPointCloudToPointCloud2(mParkingDataOnlyOneQueue,mParkingData2);
                    printf("publishing lidar started...");
                    pub_lidar.publish(mParkingData2);
                    printf("publishing lidar ended...");
                    publishHzCount = 0;

                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header = odom_buf.front()->header;
                    pose_stamped.header.frame_id = "world";
                    pose_stamped.pose = odom_buf.front()->pose.pose;
                    path.header = odom_buf.front()->header;
                    path.header.frame_id = "world";
                    path.poses.push_back(pose_stamped);
                    pub_path.publish(path);
                }
                ++publishHzCount;

                
                odom_buf.pop();
                laser_buf.pop();
            }
        }
        m_buf.unlock();

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void setParamsStatic()
{
    // if(!USE_IMU)
    // {
    //     mRLidarLeft2Right <<  1.0000000, 0.0000000, 0.0000000,
    //                         0.0000000, 1.0000000, 0.0000000,
    //                         0.0000000, 0.0000000, 1.0000000;

    //     mRLidar2Cam << 0.0000000, -1.0000000,  0.0000000,
    //                 1.0000000,  0.0000000,  0.0000000,
    //                 0.0000000,  0.0000000,  1.0000000;
        
    //     Eigen::Matrix3d eigen_T_cam2IMU;
    //     eigen_T_cam2IMU << 1.0000000,  0.0000000,  0.0000000,
    //                     0.0000000,  0.0000000,  1.0000000,
    //                     0.0000000, -1.0000000,  0.0000000;
    //     mRLidar2Cam = eigen_T_cam2IMU*mRLidar2Cam;

    //     mtLidar2Cam << 0.0380000, -0.0500000,  -0.1170000; 
    // }
    // else if(USE_IMU)
    // {
    //     mRLidarLeft2Right <<  1.0000000, 0.0000000, 0.0000000,
    //                           0.0000000, 1.0000000, 0.0000000,
    //                           0.0000000, 0.0000000, 1.0000000;

    //     mRLidar2Cam << 0.0000000, -1.0000000,  0.0000000,
    //                    1.0000000,  0.0000000,  0.0000000,
    //                    0.0000000,  0.0000000,  1.0000000;
    //     mtLidar2Cam << 0.0380000, -0.0500000,  -0.1170000; 

    //     Eigen::Matrix3d eigen_R_cam2IMU;
    //     Eigen::MatrixXd eigen_t_cam2IMU(3,1);
    //     eigen_R_cam2IMU << 0.03937385558870575, -0.012720761595540414, -0.9991435741276185, 
    //                        0.9992046897649908, -0.0058029313268805945, 0.039450144989166665, 
    //                       -0.006299797435673294, -0.9999022493286267, 0.012482160863590563;
    //     eigen_t_cam2IMU << -0.025641082939354815, -0.055552179887284316, -0.035334611107370684;
        

    //     mRLidar2Cam = eigen_R_cam2IMU*mRLidar2Cam;
    //     mtLidar2Cam = eigen_R_cam2IMU*mtLidar2Cam + eigen_t_cam2IMU;
    // }
    mRLidarLeft2Right <<  1.0000000, 0.0000000, 0.0000000,
                            0.0000000, 1.0000000, 0.0000000,
                            0.0000000, 0.0000000, 1.0000000;

    mRLidar2Cam << 0.0000000, -1.0000000,  0.0000000,
                    1.0000000,  0.0000000,  0.0000000,
                    0.0000000,  0.0000000,  1.0000000;
    mtLidar2Cam << 0.0380000, -0.0500000,  -0.1170000; 

    Eigen::Matrix3d eigen_R_cam2IMU;
    Eigen::MatrixXd eigen_t_cam2IMU(3,1);
    eigen_R_cam2IMU << -0.01326364, -0.00155043, -0.99991083, 
                       0.99990592, -0.00351886, -0.01325812,  
                       -0.003498, -0.99999261, 0.00159696;
    eigen_t_cam2IMU << -0.056674, -0.05269876, -0.04099873;

    mRLidar2Cam = eigen_R_cam2IMU*mRLidar2Cam;
    mtLidar2Cam = eigen_R_cam2IMU*mtLidar2Cam + eigen_t_cam2IMU;
}

EulerAnglesCV ToEulerAngles(QuaternionCV q) {
    EulerAnglesCV angles;
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

int saveLidarData(sensor_msgs::PointCloud &_ParkingData)
{
    pcl::PointCloud<pcl::PointXYZ> mPointcloud;

    std::string writhPath = output_path + "data3D.txt";
    std::ofstream ofile(writhPath);
    // std::ofstream ofile(writhPath,ios::app);
    std::cerr << "-------saveLidarData_path : " << writhPath << std::endl;
    if(ofile.is_open())
    {
        const int dataSize = _ParkingData.points.size();  
        ofile << std::fixed;
        ofile.precision(10);
        for(int i = 0 ; i < dataSize; ++i)
        {
            ofile << _ParkingData.points[i].x << " " << _ParkingData.points[i].y << " " << _ParkingData.points[i].z << std::endl;
        }
    }
    ofile.close();

    return 0;
}

int saveTrajectoryData(std::vector<nav_msgs::Odometry> &_TrajecotryData)
{
    std::vector<cv::Point3f> temp_coordinateX, temp_coordinateY, temp_coordinateZ;
    std::vector<cv::Point3f> convert_temp_coordinateX;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 100; ++j)
        {
            if(i == 0)
            {
                cv::Point3f tempData(j*0.01*0.1,0,0);
                temp_coordinateX.push_back(tempData);
            }else if(i == 1)
            {
                cv::Point3f tempData(0,j*0.01*0.1,0);
                temp_coordinateY.push_back(tempData);
            }else if(i == 2)
            {
                cv::Point3f tempData(0,0,j*0.01*0.1);
                temp_coordinateZ.push_back(tempData);
            }            
        }
    }    

    pcl::PointCloud<pcl::PointXYZ> mPointcloud;

    std::string writhPath = output_path + "Trajectory_data3D.txt";
    std::string writhPath_rotation = output_path + "Trajectory_data3D_rotation.txt";
    std::string writhPath_rotationVec = output_path + "Trajectory_data3D_rotation_vect.txt";
    std::string writhPath_rotationTheata = output_path + "Trajectory_data3D_rotation_theata.txt";
    
    std::ofstream ofile(writhPath);
    // std::ofstream ofile(writhPath, ios::app);
    std::ofstream ofile_rotation(writhPath_rotation);
    std::ofstream ofile_rotationVec(writhPath_rotationVec);
    std::ofstream ofile_rotationTheata(writhPath_rotationTheata);    
    if(ofile.is_open())
    {
        const size_t dataSize = _TrajecotryData.size();  
        ofile << std::fixed;
        ofile.precision(10);
        ofile_rotation << std::fixed;
        ofile_rotation.precision(10);
        ofile_rotationVec << std::fixed;
        ofile_rotationVec.precision(10);
        ofile_rotationTheata << std::fixed;
        ofile_rotationTheata.precision(10);
        for(size_t i = 0 ; i < dataSize; ++i)
        {
            if(i == 0) // Start
            {
                ofile << _TrajecotryData[i].pose.pose.position.x << " " << _TrajecotryData[i].pose.pose.position.y << " " << _TrajecotryData[i].pose.pose.position.z << " 255 0 0" << std::endl;
            }else if(i == (dataSize-1)) // Finish
            {
                ofile << _TrajecotryData[i].pose.pose.position.x << " " << _TrajecotryData[i].pose.pose.position.y << " " << _TrajecotryData[i].pose.pose.position.z << " 0 255 0" << std::endl;
            }else{
                ofile << _TrajecotryData[i].pose.pose.position.x << " " << _TrajecotryData[i].pose.pose.position.y << " " << _TrajecotryData[i].pose.pose.position.z << " " << ((255.0*i)/(float)dataSize) << " 255 255" << std::endl;                
            }
                
            // Rotation
            Eigen::Quaterniond q;
            q.x() = _TrajecotryData[i].pose.pose.orientation.x;
            q.y() = _TrajecotryData[i].pose.pose.orientation.y;
            q.z() = _TrajecotryData[i].pose.pose.orientation.z;
            q.w() = _TrajecotryData[i].pose.pose.orientation.w;
            Eigen::Matrix3d Rpose= q.normalized().toRotationMatrix();

            Eigen::MatrixXd translation_pose(3,1); 
            translation_pose << _TrajecotryData[i].pose.pose.position.x, _TrajecotryData[i].pose.pose.position.y, _TrajecotryData[i].pose.pose.position.z;
            
            cv::Point3f vectStart, vectEnd;
            {
                // Start
                {
                    Eigen::MatrixXd coordinate(3,1);                 
                    coordinate << temp_coordinateX[0].x, temp_coordinateX[0].y, temp_coordinateX[0].z;

                    Eigen::MatrixXd result_coordinate(3,1);
                    result_coordinate = Rpose*coordinate+translation_pose;
                    vectStart.x = result_coordinate(0,0);
                    vectStart.y = result_coordinate(1,0);
                    vectStart.z = result_coordinate(2,0);
                }
                // End
                {
                    Eigen::MatrixXd coordinate(3,1);                 
                    coordinate << temp_coordinateX[temp_coordinateX.size()-1].x, temp_coordinateX[temp_coordinateX.size()-1].y, temp_coordinateX[temp_coordinateX.size()-1].z;

                    Eigen::MatrixXd result_coordinate(3,1);
                    result_coordinate = Rpose*coordinate+translation_pose;
                    vectEnd.x = result_coordinate(0,0);
                    vectEnd.y = result_coordinate(1,0);
                    vectEnd.z = result_coordinate(2,0);
                }
            }
            // for(size_t icoord = 0; icoord < temp_coordinateX.size(); ++icoord)
            // {
            //     Eigen::MatrixXd coordinate(3,1);                 
            //     coordinate << temp_coordinateX[icoord].x, temp_coordinateX[icoord].y, temp_coordinateX[icoord].z;

            //     Eigen::MatrixXd result_coordinate(3,1);
            //     result_coordinate = Rpose*coordinate+translation_pose;

            //     if(i == 0) // Start
            //     {
            //         ofile_rotation << result_coordinate(0,0) << " " << result_coordinate(1,0) << " " << result_coordinate(2,0) << " 255 0 0" << std::endl;                    
            //         vectStart.x = result_coordinate(0,0);
            //         vectStart.y = result_coordinate(1,0);
            //         vectStart.z = result_coordinate(2,0);
            //     }else if(i == (dataSize-1)) // Finish
            //     {
            //         ofile_rotation << result_coordinate(0,0) << " " << result_coordinate(1,0) << " " << result_coordinate(2,0) << " 0 255 0" << std::endl;                    
            //         vectEnd.x = result_coordinate(0,0);
            //         vectEnd.y = result_coordinate(1,0);
            //         vectEnd.z = result_coordinate(2,0);
            //     }else
            //     {
            //         ofile_rotation << result_coordinate(0,0) << " " << result_coordinate(1,0) << " " << result_coordinate(2,0) << " 255 255 255" << std::endl;
            //     }
            // }

            // RotationVec
            {
                cv::Point3f temp_coordinateVecX;
                temp_coordinateVecX = vectEnd-vectStart;
                convert_temp_coordinateX.push_back(temp_coordinateVecX);
                ofile_rotationVec << temp_coordinateVecX.x << " " << temp_coordinateVecX.y << " " << temp_coordinateVecX.z << std::endl;
            }
        }

        // RotationTheata
        {
            const cv::Point3f last_vectorX = convert_temp_coordinateX[convert_temp_coordinateX.size()-1];
            for(size_t i = 0; i < convert_temp_coordinateX.size(); ++i)
            {
                cv::Point3f curr_vectorX = convert_temp_coordinateX[i];
                float dot_product_value = curr_vectorX.x*last_vectorX.x + curr_vectorX.y*last_vectorX.y + curr_vectorX.z*last_vectorX.z;
                float scale_curr = std::sqrt(curr_vectorX.x*curr_vectorX.x + curr_vectorX.y*curr_vectorX.y + curr_vectorX.z*curr_vectorX.z);
                float scale_last = std::sqrt(last_vectorX.x*last_vectorX.x + last_vectorX.y*last_vectorX.y + last_vectorX.z*last_vectorX.z);
                float cos_theata = dot_product_value / (scale_curr*scale_last);
                ofile_rotationTheata << i << " " << cos_theata << std::endl;
            }
        }
    }
    ofile.close();
    ofile_rotation.close();
    ofile_rotationVec.close();
    ofile_rotationTheata.close();

    return 0;
}

// extract images with same timestamp from two topics
void save_process()
{
    printf("-------------- [SAVE PROCESS ON] -------------");
    while(1)
    {        
        char saveFlag;
        std::cin >> saveFlag;
        // printf("-------------- [SAVE PROCESS WHILE ON] -------------");

        if(saveFlag == 's')
        // if(1)
        {
            printf("-------------- [SAVE PROCESS IF ON] -------------");

            m_buf.lock();
            // std::cerr << "[SAVE] Write 3D-data" << std::endl;
            // ROS_WARN("[SAVE] Write 3D-data");
            saveLidarData(mParkingData);
            saveTrajectoryData(odom_data_vector);
            std::cerr << "[SAVE] Finish 3D-data" << std::endl;
            m_buf.unlock();
            signal(SIGINT, SIG_DFL);
            break;            
        }        

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosLidarNode");
    ros::NodeHandle n("~");
    string config_file;        
    n.getParam("USE_IMU", USE_IMU);
    n.getParam("config_file", config_file);    

    readParameters(config_file);
    //setParamsStatic();

    mParkingData2.header.frame_id = "world";
    mParkingData2.height = 3;
    mParkingData2.width = 0;   

    // pubish
    pub_lidar = n.advertise<sensor_msgs::PointCloud2>("/parking/scan", 1);
    pub_path = n.advertise<nav_msgs::Path>("/parking/path", 1);

    // subscribe
    ros::Subscriber sub_odom  = n.subscribe("/vins_estimator/odometry", 1, odom_callback);
    // ros::Subscriber sub_odom  = n.subscribe("/refined_odom", 1, odom_callback);
    ros::Subscriber sub_laser = n.subscribe("/scan", 1, laser_callback);
    
    std::thread sync_thread{sync_process};
    std::thread save_thread{save_process};
    printf("++++++++++++++++++++++++++ roslidarnode spins... +++++++++++++++++++++++++");
    ros::spin();
    
    return 0;
}

void readParameters(std::string config_file) // modify_crvl
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        // ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["vio_lidar_3d_map_path"] >> VIO_LIDAR_3D_MAP_PATH;

    cv::Mat cv_imu_T_body;
    fsSettings["imu_T_body"] >> cv_imu_T_body;
    
    //fsSettings["output_path"] >> output_path;
    // std::string env_var_output_path=(getenv("ParkingRamp3D_PATH")); // 원랜 주석 아님
    std::string env_var_output_path=VIO_LIDAR_3D_MAP_PATH;    //내가추가

    output_path = env_var_output_path;    //내가추가
    std::cerr << "lidar_data_output_path: " << output_path << std::endl;

    // ROS_WARN("output_path: "+ output_path);
    Eigen::Matrix4d T;
    cv::cv2eigen(cv_imu_T_body, T);
    Eigen::Matrix3d eigen_R_cam2IMU;
    Eigen::MatrixXd eigen_t_cam2IMU(3,1);

    eigen_R_cam2IMU = (T.block<3, 3>(0, 0));
    eigen_t_cam2IMU = (T.block<3, 1>(0, 3));
          

    fsSettings.release();

    //======================================================
    mRLidarLeft2Right <<  1.0000000, 0.0000000, 0.0000000,
                            0.0000000, 1.0000000, 0.0000000,
                            0.0000000, 0.0000000, 1.0000000;

    mRLidar2Cam << 0.0000000, -1.0000000,  0.0000000,
                    1.0000000,  0.0000000,  0.0000000,
                    0.0000000,  0.0000000,  1.0000000;
    mtLidar2Cam << 0.0380000, -0.0500000,  -0.1170000; 

    mRLidar2Cam = eigen_R_cam2IMU*mRLidar2Cam;
    mtLidar2Cam = eigen_R_cam2IMU*mtLidar2Cam + eigen_t_cam2IMU;
}
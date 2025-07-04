/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "parameters.h"


double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<cv::Mat> intrinsic3x3;
std::vector<cv::Mat> distortion5x1;

std::vector<cv::Mat> stereoRectify_R;
std::vector<cv::Mat> stereoRectify_P;

std::vector<cv::Point2f> distortedImageMapRight;
std::vector<cv::Point2f> undistortedCameraPlaneMapRight;
kdt::KDTree<kdt::CustomKDTreePoint> kdtree_undistorted;
std::vector<kdt::CustomKDTreePoint> kdtree_undistortedProjPts;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;
Eigen::Matrix3f essentialMatrix;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string OUTPUT_FOLDER;
std::string IMU_TOPIC;
int ROW, COL;
double TD;
int NUM_OF_CAM;
int STEREO;
int USE_IMU;
int USE_SEG;
int MULTIPLE_THREAD;
map<int, Eigen::Vector3d> pts_gt;
std::string IMAGE0_TOPIC, IMAGE1_TOPIC;
std::string FISHEYE_MASK;
std::vector<std::string> CAM_NAMES;
int MAX_CNT;
int MIN_DIST;
double F_THRESHOLD;
int SHOW_TRACK;
int FLOW_BACK;

float mStereoKernelShiftY;

//sivvon
std::string SP_SG_DATA_DIR;
float KP_THRESHOLD;
float TRACK_THRESHOLD; 
float MATCH_THRESHOLD;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void readParameters_origin(std::string config_file)
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }
    

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();

}

void readParameters_crvl(std::string config_file) // modify_crvl
{
    FILE *fh = fopen(config_file.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK();
        return;          
    }
    fclose(fh);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["image0_topic"] >> IMAGE0_TOPIC;
    fsSettings["image1_topic"] >> IMAGE1_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    FLOW_BACK = fsSettings["flow_back"];

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    //sivvon
    fsSettings["superpoint_superglue_kp_track_match_data"] >> SP_SG_DATA_DIR;
    KP_THRESHOLD = fsSettings["cam0_cur_keypoint_confidence_threshold"];
    TRACK_THRESHOLD = fsSettings["cam0_tracking_confidence_threshold"]; 
    MATCH_THRESHOLD = fsSettings["stereo_matching_confidence_threshold"];

    MULTIPLE_THREAD = fsSettings["multiple_thread"];

    //cv::Mat cv_T_cam2IMU = cv::Mat::zeros(4,4,CV_64F);
    cv::Mat cv_T_cam2IMU = cv::Mat::eye(4,4,CV_64F);

    USE_IMU = fsSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        fsSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = fsSettings["acc_n"];
        ACC_W = fsSettings["acc_w"];
        GYR_N = fsSettings["gyr_n"];
        GYR_W = fsSettings["gyr_w"];
        G.z() = fsSettings["g_norm"];
    }
    if(!USE_IMU)
    {
        cv::Mat cv_imu_T_body;
        fsSettings["imu_T_body"] >> cv_imu_T_body;
        cv_T_cam2IMU = cv_imu_T_body* cv_T_cam2IMU;
            
        // cv_T_cam2IMU.at<double>(0,0) = 1;
        // cv_T_cam2IMU.at<double>(1,2) = 1;
        // cv_T_cam2IMU.at<double>(2,1) = -1;
        // cv_T_cam2IMU.at<double>(3,3) = 1;
    }

    mStereoKernelShiftY = fsSettings["stereoKernelShiftY"];
    USE_SEG = fsSettings["seg"]; 

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    fsSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    // std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    // std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    // fout.close();

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        if(!USE_IMU)
            cv_T = cv_T_cam2IMU*cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    
    NUM_OF_CAM = fsSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }



    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    std::cerr << "cam0Path: " << cam0Path << std::endl;
    {
        cv::FileStorage fsSettingsCam(cam0Path, cv::FileStorage::READ);
        if(!fsSettingsCam.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }else
        {
            cv::Mat intrinsicMatrix  = cv::Mat::eye(3, 3, cv::DataType<double>::type);
            cv::Mat distCoeffsMatrix = cv::Mat::zeros(5, 1, cv::DataType<double>::type);
            intrinsicMatrix.ptr<double>(0,0)[0] = (double)fsSettingsCam["projection_parameters"]["fx"];
            intrinsicMatrix.ptr<double>(1,1)[0] = (double)fsSettingsCam["projection_parameters"]["fx"];
            intrinsicMatrix.ptr<double>(0,2)[0] = (double)fsSettingsCam["projection_parameters"]["cx"];
            intrinsicMatrix.ptr<double>(1,2)[0] = (double)fsSettingsCam["projection_parameters"]["cy"];

            distCoeffsMatrix.ptr<double>(0,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k1"];
            distCoeffsMatrix.ptr<double>(1,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k2"];        
            distCoeffsMatrix.ptr<double>(2,0)[0] = (double)fsSettingsCam["distortion_parameters"]["p1"];
            distCoeffsMatrix.ptr<double>(3,0)[0] = (double)fsSettingsCam["distortion_parameters"]["p2"];
            distCoeffsMatrix.ptr<double>(4,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k3"];
            intrinsic3x3.push_back(intrinsicMatrix);
            distortion5x1.push_back(distCoeffsMatrix);
        }
        fsSettingsCam.release();
    }
    

    if(NUM_OF_CAM == 2)
    {
        cv::Mat cv_T;
        fsSettings["body_T_cam1"] >> cv_T;
        if(!USE_IMU)
        {
            cv::Mat R_Cam1toCam0 = cv_T.rowRange(0,3).colRange(0,3).clone();
            cv::Mat t_Cam1toCam0 = cv_T.rowRange(0,3).colRange(3,4).clone();
            cv::Mat t_Cam1toCam0x = R_Cam1toCam0.clone();
            t_Cam1toCam0x.ptr<double>(0,0)[0] = 0;
            t_Cam1toCam0x.ptr<double>(0,1)[0] = -1*t_Cam1toCam0.ptr<double>(2,0)[0];
            t_Cam1toCam0x.ptr<double>(0,2)[0] = t_Cam1toCam0.ptr<double>(1,0)[0];

            t_Cam1toCam0x.ptr<double>(1,0)[0] = t_Cam1toCam0.ptr<double>(2,0)[0];
            t_Cam1toCam0x.ptr<double>(1,1)[0] = 0;
            t_Cam1toCam0x.ptr<double>(1,2)[0] = -1*t_Cam1toCam0.ptr<double>(0,0)[0];

            t_Cam1toCam0x.ptr<double>(2,0)[0] = -1*t_Cam1toCam0.ptr<double>(2,0)[0];
            t_Cam1toCam0x.ptr<double>(2,1)[0] = t_Cam1toCam0.ptr<double>(1,0)[0];
            t_Cam1toCam0x.ptr<double>(2,2)[0] = 0;

            cv::Mat cv_essentialMatrix = t_Cam1toCam0x.mul(R_Cam1toCam0);
            cv::cv2eigen(cv_essentialMatrix, essentialMatrix);
            
            cv_T = cv_T_cam2IMU*cv_T;
        }

        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));

        STEREO = 1;
        std::string cam1Calib;
        fsSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        {
            cv::FileStorage fsSettingsCam(cam1Path, cv::FileStorage::READ);
            if(!fsSettingsCam.isOpened())
            {
                std::cerr << "ERROR: Wrong path to settings" << std::endl;
            }else
            {
                cv::Mat intrinsicMatrix  = cv::Mat::eye(3, 3, cv::DataType<double>::type);
                cv::Mat distCoeffsMatrix = cv::Mat::zeros(5, 1, cv::DataType<double>::type);
                intrinsicMatrix.ptr<double>(0,0)[0] = (double)fsSettingsCam["projection_parameters"]["fx"];
                intrinsicMatrix.ptr<double>(1,1)[0] = (double)fsSettingsCam["projection_parameters"]["fx"];
                intrinsicMatrix.ptr<double>(0,2)[0] = (double)fsSettingsCam["projection_parameters"]["cx"];
                intrinsicMatrix.ptr<double>(1,2)[0] = (double)fsSettingsCam["projection_parameters"]["cy"];

                distCoeffsMatrix.ptr<double>(0,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k1"];
                distCoeffsMatrix.ptr<double>(1,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k2"];        
                distCoeffsMatrix.ptr<double>(2,0)[0] = (double)fsSettingsCam["distortion_parameters"]["p1"];
                distCoeffsMatrix.ptr<double>(3,0)[0] = (double)fsSettingsCam["distortion_parameters"]["p2"];
                distCoeffsMatrix.ptr<double>(4,0)[0] = (double)fsSettingsCam["distortion_parameters"]["k3"];
                intrinsic3x3.push_back(intrinsicMatrix);
                distortion5x1.push_back(distCoeffsMatrix);
            }
            fsSettingsCam.release();
        }
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    fsSettings.release();
}
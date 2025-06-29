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

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

//SIVVON
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
//SIVVON
#include <dirent.h>
#include <cstring>
#include <algorithm>
#include <regex>
#include <utility>

#include <iomanip>
#include <string>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"
#include "stereoBlockMatching.h"

#define FRAME_GRID_ROWS 30
#define FRAME_GRID_COLS 60
#define NUM_OF_GRID     5

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<bool> status);

class FeatureTracker
{
public:
    FeatureTracker();
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageBlockMatching_origin(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageBlockMatching(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat(), int _seq = 0);
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageMonoORB(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageORB(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
    void setMask();
    void readIntrinsicParameter(const vector<string> &calib_file);
    void showUndistortion(const string &name);
    void rejectWithF();
    void undistortedPoints();
    void epipolarConstrantOutlierRemover(vector<cv::Point2f> &_unLeftpts, vector<cv::Point2f> &_unRightpts, vector<uchar> &_status, const Eigen::Matrix3f &_essentialMatrix, vector<cv::Point2f> &_cur_pts);
    vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
    vector<cv::Point2f> undistortedPts(vector<cv::KeyPoint> &pts, camodocal::CameraPtr cam);
    //vector<cv::Point2f> undistortedPtsWidthOutDistortion(vector<cv::Point2f> &pts);
    vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                    map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);    
    void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
                      vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::Point2f> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                                   vector<int> &curLeftIds,
                                   vector<cv::KeyPoint> &curLeftPts, 
                                   vector<cv::Point2f> &curRightPts,
                                   map<int, cv::Point2f> &prevLeftPtsMap);
    void setPrediction(map<int, Eigen::Vector3d> &predictPts);
    double distance(cv::Point2f &pt1, cv::Point2f &pt2);
    void removeOutliers(set<int> &removePtsIds);
    cv::Mat getTrackImage();
    bool inBorder(const cv::Point2f &pt);
    //SIVVON
    void TrackorMatchwithSPnSG(vector<cv::Point2f> &prev_pts,
                               vector<cv::Point2f> &cur_pts,
                               vector<cv::Point2f> &prevXY,
                               vector<cv::Point2f> &curXY,
                               vector<float> &Confidence,
                               vector<uchar> &status,
                               const float Threshold
    );
    //SIVVON
    void SuperPointsToTrack(vector<cv::Point2f> &n_pts,
                            vector<pair<float, cv::Point2f>> &curSuperPointFeature,
                            cv::Mat mask,
                            float KpThreshold,
                            int n_max_cnt
    );
    
    template <typename T>
    void copyVector(std::vector<T>& src, std::vector<T>& target);

    int row, col;
    cv::Mat imTrack;
    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> predict_pts;
    vector<cv::Point2f> predict_pts_debug;
    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
    vector<cv::Point2f> pts_velocity, right_pts_velocity;
    vector<int> ids, ids_right, ids_prev;
    vector<int> track_cnt, track_cnt_prev;
    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
    map<int, cv::Point2f> prevLeftPtsMap;
    vector<camodocal::CameraPtr> m_camera;
    double cur_time;
    double prev_time;
    bool stereo_cam;
    int n_id;
    bool hasPrediction;

    // intrinsic matrix
    //Matrix3d instrinsic, instrinsic_inverse;

    // For ORB feature
    // Initialize keypoints, descriptors and type of descriptor
    std::vector<cv::KeyPoint> keypointsORB_L_prev, keypointsORB_L, keypointsORB_R; // 정렬
    cv::Mat descriptorsORB_L_prev, descriptorsORB_L, descriptorsORB_R;             // 정렬

    std::vector<cv::KeyPoint> keypointsORB_L_prev2, keypointsORB_L2; // 초기
    cv::Mat descriptorsORB_L_prev2, descriptorsORB_L2;               // 초기
    
    cv::Ptr<cv::Feature2D> ftrD_ORB;
    cv::Ptr<cv::Feature2D> ftrD_ORB_right;

    cv::Ptr<cv::DescriptorMatcher> descriptorORBMatcher;
    // std::vector<cv::DMatch> matchesORB_seq, bestMatchesORB_seq;
    // std::vector<cv::DMatch> matchesORB_stereo, bestMatchesORB_stereo;

    std::vector<cv::DMatch> bestMatchesORB_seq;
    std::vector<cv::DMatch> bestMatchesORB_stereo;
    std::vector<std::vector<cv::DMatch>> matchesORB_seq;
    std::vector<std::vector<cv::DMatch>> matchesORB_stereo;
    cv::Mat matchingImg_ORB;

    std::vector<std::size_t> mGrid[FRAME_GRID_ROWS][FRAME_GRID_COLS];
    std::vector<float> mGrid_Score[FRAME_GRID_ROWS][FRAME_GRID_COLS];
    std::vector<std::size_t> mGrid_remove_index;
    void gridSampling();
};

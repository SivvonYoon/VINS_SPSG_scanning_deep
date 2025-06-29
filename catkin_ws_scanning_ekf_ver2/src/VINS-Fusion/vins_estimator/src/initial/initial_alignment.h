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
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../estimator/feature_manager.h"

using namespace Eigen;      // Eigen : 행렬 및 벡터 연산과 같이 선형 연산과 관련도니 함수와 클랫므를 지원하는 라이브러리.
using namespace std;        // 헤더 파일(h or hpp) 에서 이렇게 선언하면 해당 헤더 파일을 포함하는 cpp 파일에도 적용됨. 그러나 헤더 파일에 선언하는 것은 권장되지 않는 방식.

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;           // 시간 또는 타임 스탬프를 나타내는 변수
        Matrix3d R;         // rotation
        Vector3d T;         // translation
        IntegrationBase *pre_integration;
        bool is_key_frame;      // 해당 프레임이 키프레임인지 아닌지 확인
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
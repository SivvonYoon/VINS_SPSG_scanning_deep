#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include <eigen3/Eigen/Dense>
#include "kdtree.h"

void calStereoBlockMinScoreSortingPatch(
    const cv::Mat &leftImg,
    const cv::Mat &rightImg,
    const int &indexLeftPts,
    const std::vector<cv::Point2f> &leftPts,
    const std::vector<cv::Point2f> &candidate_rightPts,    
    const std::vector<int> &indices,
    const int &mBlockMatching,
    const int &mBlockMatchingKernalSize,
    const float &mStereoSearchingPaddingX,
    const float &mStereoSearchingPaddingY,
    const float &mStereoKernelShiftY,
    float &minScore, int &minIndex);

void calcStereoMatchingGoodFeature2Track(
    const cv::Mat &leftImg,
    const cv::Mat &rightImg,
    const std::vector<cv::Point2f> &leftPts,
    std::vector<cv::Point2f> &rightPts,
    std::vector<uchar> &status,
    std::vector<float> &err,
    const cv::Size winSize,
    const float maxThreshold ,
    const float stereoKernelShiftY
    );

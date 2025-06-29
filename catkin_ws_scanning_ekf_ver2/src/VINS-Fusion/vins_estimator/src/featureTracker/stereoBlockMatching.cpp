#include "stereoBlockMatching.h"

inline float 
// 
// 선형 보간법 : 알려진 범위 내의 값 추정 --> 예시 : 20세 키와 40세 키를 알 때 30세 키 추정
BilinearInterpolation(float q11, float q12, float q21, float q22, float x1, float x2, float y1, float y2, float x, float y) 
// q11, q12, q21, q22 --> 픽셀 값 (getPatchFromMat 참고)
{    
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;         // x 좌표 범위 : x2(x 가장 큰 좌표값), x1(x 가장 작은 좌표)
    y2y1 = y2 - y1;         // y 좌표 범위 : y2(y 가장 큰 좌표), y1(y 가장 작은 좌표)
    x2x = x2 - x;           // x2x : x 범위 중 가장 큰 값(x2)에서 목표좌표(x,y)의 x좌표(x)까지의 거리 , x : 범위 내에 있는 목표 좌표의 x 좌표값
    y2y = y2 - y;           // y2y : y 범위 중 가장 큰 값(y2)에서 목표좌표(x,y)의 y좌표(y)까지의 거리 , y : 범위 내에 있는 목표 좌표의 y 좌표값
    yy1 = y - y1;           // yy1 : y 범위 중 가장 작은 값(y1)에서 목표좌표(x,y)의 y좌표(y)까지의 거리 , y : 범위 내에 있는 목표 좌표의 y 좌표값
    xx1 = x - x1;           // xx1 : x 범위 중 가장 작은 값(x1)에서 목표좌표(x,y)의 x좌표(x)까지의 거리 , x : 범위 내에 있는 목표 좌표의 x 좌표값
    return 1.0 / (x2x1*y2y1)*(q11*x2x*y2y+q21*xx1*y2y+q12*x2x*yy1+q22*xx1*yy1);
}

// Patch 구하는 코드 : 중심 x,y 기준으로 +-1 씩 각 변 길이가 2인 patch
inline void getPatchFromMat(const cv::Mat img, const float centerX, const float centerY, const int patchSize, float *patch)
{
    const uchar* img_data = img.data;                   // img.data --> img(input1) 의 pixel data

    const int centerXi = (int)centerX;                  // centerX : patch 중심점의 x 좌표
    const int centerYi = (int)centerY;                  // centerY : patch 중심점의 y 좌표

    for(int r = 0; r < patchSize; ++r)                  // r : row, patchSize : patch 가로세로 사이즈
    {
        for(int c = 0; c < patchSize; ++c)              // c : cloumn, patchSize : patch 가로세로 사이즈
        {
            // 
            /* 
            img.ptr --> 이미지 픽셀 데이터 포인터
            img.ptr<uchar>(y,x) --> (y,x) 에 해당하는 픽셀 값
            img.ptr<uchar>(y,x)[0] --> RGB 채널이었으면 3채널이어서 의미가 있을 텐데, 그레이스케일은 어차피 채널 하나여서 의미없음. 그냥 img.ptr<uchar>(y,x) 랑 똑같음.
            */
            patch[r*patchSize + c] = BilinearInterpolation(img.ptr<uchar>(centerYi - 1,centerXi - 1)[0],
                                                           img.ptr<uchar>(centerYi + 1,centerXi - 1)[0],
                                                           img.ptr<uchar>(centerYi - 1,centerXi + 1)[0],
                                                           img.ptr<uchar>(centerYi + 1,centerXi + 1)[0],
                                                           centerXi - 1, centerXi + 1,
                                                           centerYi - 1, centerYi + 1,
                                                           centerXi, centerYi);
        }
    }
}

float blockMatchingPatch_CensusTransformContinues(const float *patchl, const float *patchr, const int w_size)
{
    const int centerIndex = w_size * 0.5;
    const int censusCenter = patchl[centerIndex*w_size + centerIndex];
    int censusl = 0;
    int censusr = 0;
    int censusCost = 0;

    int continuesCost = 0;
    int censusCostCell[w_size*w_size];

    // Census
    for(int i = 0; i < w_size; ++i)
    {
        for(int j = 0; j < w_size; ++j)
        {
            censusl = (censusCenter > patchl[j*w_size+i]) ? (1) : (0);
            censusr = (censusCenter > patchr[j*w_size+i]) ? (1) : (0);
            censusCostCell[j*w_size+i] = ((censusl == censusr) ? (1) : (0));
            censusCost += censusCostCell[j*w_size+i];
        }
    }

    // continus check
    int currValue = censusCostCell[0];
    int maxContinusCost = (w_size-1)*4;
    int start = 0;
    int end = (w_size-1)*4;

    // backward
    while(start != end)
    {
        if (currValue == censusCostCell[end])
        {
            ++continuesCost;
            --end;
        }else
        {
            break;
        }
    }

    if(currValue == censusCostCell[1])
    {
        //forward
        while(start != end)
        {
            if (currValue == censusCostCell[start])
            {
                ++continuesCost;
            }else
            {
                if(maxContinusCost < continuesCost)
                {
                    maxContinusCost = continuesCost;
                    currValue = censusCostCell[start];
                }
                continuesCost = 0;
            }
            ++start;
        }
    }else
    {
        maxContinusCost = continuesCost;             // maxContinusCost: 비용 --> 비용이 높을수록 유사한 patch
        continuesCost = 0;

        //forward
        while(start != end)
        {
            if (currValue == censusCostCell[start])
            {
                ++continuesCost;
            }else
            {
                if(maxContinusCost < continuesCost)
                {
                    maxContinusCost = continuesCost;
                    currValue = censusCostCell[start];
                }
                continuesCost = 0;
            }
            ++start;
        }
    }
    
    continuesCost = ((w_size-1) * 4) - maxContinusCost;        
    
    //return censusCost + continuesCost * 0.25;
    return continuesCost;
}

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
    float &minScore, int &minIndex)
{
    std::vector<float> error_score_candidate(indices.size(), 9999999999);
    cv::Point2f pl, pr;
    float patchl[mBlockMatchingKernalSize*mBlockMatchingKernalSize];
    float patchr[mBlockMatchingKernalSize*mBlockMatchingKernalSize];
    getPatchFromMat(leftImg, leftPts[indexLeftPts].x, leftPts[indexLeftPts].y, mBlockMatchingKernalSize, patchl);

    pl = cv::Point2f(leftPts[indexLeftPts].x, leftPts[indexLeftPts].y);
    for(size_t index=0, iend=indices.size(); index < iend; ++index)
    {        
        const float distX = leftPts[indexLeftPts].x-candidate_rightPts[indices[index]].x;
        const float distY = std::abs((leftPts[indexLeftPts].y-mStereoKernelShiftY)-candidate_rightPts[indices[index]].y);

        if((distX > 0) &&
           (distX <= mStereoSearchingPaddingX) &&
           (distY <= mStereoSearchingPaddingY))
        {            
            getPatchFromMat(rightImg, candidate_rightPts[indices[index]].x, candidate_rightPts[indices[index]].y, mBlockMatchingKernalSize, patchr);
            error_score_candidate[index] = blockMatchingPatch_CensusTransformContinues(patchl, patchr, mBlockMatchingKernalSize);
        }else
        {
            error_score_candidate[index] = 9999999999;
        }
    }
    minScore = *std::min_element(error_score_candidate.begin(), error_score_candidate.end());
    minIndex = std::min_element(error_score_candidate.begin(), error_score_candidate.end())-error_score_candidate.begin();
}



void calcStereoMatchingGoodFeature2Track(
    const cv::Mat &leftImg,
    const cv::Mat &rightImg,
    const std::vector<cv::Point2f> &leftPts,
    std::vector<cv::Point2f> &rightPts,
    std::vector<uchar> &status,
    std::vector<float> &err,
    const cv::Size winSize = cv::Size(21, 21),
    const float maxThreshold = 30,
    const float stereoKernelShiftY = 0
    )
{
    rightPts.resize(leftPts.size());
    status.resize(leftPts.size());
    err.resize(leftPts.size());

    std::vector<cv::Point2f> candidate_rightPts;
    float error_score_stereoBlockMatching = 0;
    std::vector<kdt::CustomKDTreePoint> candidate_rightPtsKDtree;

    cv::goodFeaturesToTrack(rightImg, candidate_rightPts, 20000, 0.01, 0.2);
    candidate_rightPtsKDtree.resize(candidate_rightPts.size());
    for(size_t i=0,iend=candidate_rightPtsKDtree.size(); i < iend; ++i)
    {
        candidate_rightPtsKDtree[i] = kdt::CustomKDTreePoint(candidate_rightPts[i].x, candidate_rightPts[i].y);
    }
    
    kdt::KDTree<kdt::CustomKDTreePoint> kdtree(candidate_rightPtsKDtree);
    kdt::CustomKDTreePoint query;
    std::vector<int> indices;
    std::vector<float> dists;
    const double radius= winSize.height;    
    
    for(size_t indexLeftPts=0, iend=leftPts.size(); indexLeftPts < iend; ++indexLeftPts)
    {
        query = kdt::CustomKDTreePoint(leftPts[indexLeftPts].x, leftPts[indexLeftPts].y-stereoKernelShiftY);
        indices.clear();
        dists.clear();
	    indices = kdtree.radiusSearch(query, radius);

        if(indices.size() > 0)
        {
            float minScore = 0;
            int   minIndex = 0;
            const int KernalSize=5;
            calStereoBlockMinScoreSortingPatch(
                                            leftImg,
                                            rightImg,
                                            indexLeftPts,
                                            leftPts,
                                            candidate_rightPts,    
                                            indices,
                                            7, // SAD(0), NCC(1), SSD(2), ZSSD(3), Census(5), NCC+Census(6), CensusContinue(7)
                                            KernalSize, // KernalSize
                                            21, // stereoSearchingPaddingX
                                            5,  // stereoSearchingPaddingY
                                            stereoKernelShiftY,
                                            minScore, minIndex);
            
            if(minScore <= (KernalSize*0.6)) // CensusContinues
            {
                rightPts[indexLeftPts] = candidate_rightPts[indices[minIndex]];
                status[indexLeftPts] = 1;
                err[indexLeftPts]    = 0;
            }
            else
            {
                status[indexLeftPts] = 0;
                err[indexLeftPts]    = 0;
            }
        }
    }
}
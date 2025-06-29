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

#include "feature_tracker.h"

template <typename T>       // 자료형을 모호하게 둔다. (정하지 않음)
// input1 vector(src) 를 복사해 input2 vector(target) 에 저장하는 함수.
void FeatureTracker::copyVector(std::vector<T>& src, std::vector<T>& target)
{
    target.clear();                                         // size 를 0 으로 만들어 준다.
    target.resize((int)src.size());                         // 'target(input2)' vector 의 size 를 'src(input1)' 의 size 로 resize 한다.
    std::copy(src.begin(), src.end(), target.begin());      // 'src(input1)' vector 를 'target(input2)' vector 로 복사한다.
    // (std::copy --> 배열을 편리하게 복사하는 함수)
    // std::copy(first, last, target_first) --> first, last : 원소들의 범위를 나타내는 반복자, target_first : 복사할 원소들을 저장할 곳의 시작점을 나타내는 반복자
}

// Point2f 형태의 input pt (feature point) 가 이미지 내부에 존재하는지 확인하는 함수.
bool FeatureTracker::inBorder(const cv::Point2f &pt)        // pt --> (float pt.x, float pt.y)
{
    const int BORDER_SIZE = 1;      // border(테두리) size 를 1 로 지정한다.
    int img_x = cvRound(pt.x);      // 실수 형태인 pt.x 값을 반올림해 정수 형태로 img_x(int) 에 저장한다.
    int img_y = cvRound(pt.y);      // 실수 형태인 pt.y 값을 반올림해 정수 형태로 img_y(int) 에 저장한다.
    return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < row - BORDER_SIZE;  // 0(False) 또는 1(True) 반환
    // ( 조건1 ) BORDER_SIZE <= img_x : img_x 즉 pt.x 좌표값의 정수형이 아무리 작아도 처음 시작 테두리 안쪽에 있어야 한다.
    // ( 조건2 ) img_x < col - BORDER_SIZE : img_x 가 아무리 커도 제일 큰 좌표 부근의 테두리 좌표보다는 값이 작아야 한다.
    // ( 조건3 ) BORDER_SIZE <= img_y : ( 조건1 ) 과 유사함.
    // ( 조건4 ) img_y < row - BORDER_SIZE : ( 조건2 ) 와 유사함.
}

// 두 프레임의 feature point 유클리드 거리를 계산하는 함수.
// 프레임1 과 프레임2 는 각각 좌우 stereo 이미지가 될 수도 있고, 전후 프레임이 될 수도 있다.
double distance(cv::Point2f pt1, cv::Point2f pt2)       
// input1 ; pt1 (프레임1 의 feature point), input2 ; pt2 (프레임2 의 feature point)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);        // pt print test. 내가 테스트한 거 아님.
    double dx = pt1.x - pt2.x;      // dx : pt1(input1)의 x 좌표와 pt2(input2) 의 x 좌표간 거리를 구함. 각 좌표값은 원래 4바이트 float 형이나, 거리는 8바이트 double 형임.
    double dy = pt1.y - pt2.y;      // dy : pt1(input1)의 y 좌표와 pt2(input2) 의 y 좌표간 거리를 구함.
    return sqrt(dx * dx + dy * dy); // 유클리드 거리로 pt1(input1) 과 pt2(input2) 의 distance 를 return
}

// v(input1) vector 에 포함되는 요소들 중 status(input2) 가 1 인 것들만 남도록 변환하는 함수.
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;                                  // j : status(input2) vector 의 요소 중 값이 1인 것들의 순서
    for (int i = 0; i < int(v.size()); i++)     // i : v(input1) 내부 요소의 순서 (v (input1) size 만큼 반복)
        if (status[i])                          // if status[i] == True
            v[j++] = v[i];                      // 갱신 (j 는 무조건 i 보다 같거나 작으므로)
    v.resize(j);
}

// 위 함수와 동일하나, v(input1) vector 의 요소 type 이 float(실수형) 이 아닌 int(정수형).
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// 위 두 함수와 기능상으로는 동일하나, status(input2) vector 의 요소 type 이 uchar(unsigned char, 즉 True or False) 이 아닌 boolean(0 or 1) 형태.
void reduceVector(vector<int> &v, vector<bool> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

// 초기화부분만 남기고 주석처리 되어 있다. (처음부터 이 상태였음.)
FeatureTracker::FeatureTracker()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;

    // const int   orb_params_nfeatures = 500;
    // const float orb_params_scaleFactor = 1.2f;
    // const int   orb_params_nlevels = 4;
    // const int   orb_params_edgeThreshold = 31;
    // const int   orb_params_firstLevel = 0;
    // const int   orb_params_WTA_K = 2;
    // const int   orb_params_scoreType = cv::ORB::HARRIS_SCORE; //cv::ORB::FAST_SCORE, cv::ORB::HARRIS_SCORE;
    // const int   orb_params_patchSize = 31;
    // const int   orb_params_fastThreshold =20;
    // ftrD_ORB = cv::ORB::create(orb_params_nfeatures, orb_params_scaleFactor, orb_params_nlevels,
    //                            orb_params_edgeThreshold, orb_params_firstLevel, orb_params_WTA_K,
    //                            orb_params_scoreType, orb_params_patchSize, orb_params_fastThreshold);

    // const int   orb_params_nfeatures_right = 2000;
    // ftrD_ORB_right = cv::ORB::create(orb_params_nfeatures, orb_params_scaleFactor, orb_params_nlevels,
    //                            orb_params_edgeThreshold, orb_params_firstLevel, orb_params_WTA_K,
    //                            orb_params_scoreType, orb_params_patchSize, orb_params_fastThreshold);
    // // ftrD_ORB = cv::ORB::create();
    // descriptorORBMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming"); //FlannBased or BruteForce-Hamming
    // //descriptorORBMatcher = cv::DescriptorMatcher::create("FlannBased"); //FlannBased or BruteForce-Hamming
    

    // // ftrD_ORB = cv::xfeatures2d::SURF::create();
    // // descriptorORBMatcher = cv::DescriptorMatcher::create("FlannBased"); //FlannBased or BruteForce-Hamming
}

// track_cnt, cur_pts, ids 로 구성된 cnt_pts_id 를 구성하고 내림차순 정렬한다.
// 이후 정렬된 cnt_pts_id 를 이용해 좌표 픽셀값이 255인 경우에 한해 track_cnt, cur_pts, ids 를 재구성하는 함수.
// 이미지에 재구성한 좌표들을 표시한다. (cv::circle)
void FeatureTracker::setMask()
{
    mask = cv::Mat(row, col, CV_8UC1, cv::Scalar(255));     // row : cur_img(cam0 image).rows, col : cur_img(cam0 image).cols (TrackImage()에서 선언됨.)

    // prefer to keep features that are tracked for long time

    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;   // cnt_pts_id : feature point id 를 세는 변수
    // pair type : 서로 연관된 다른 타입의 데이터를 묶고 싶을 때 사용
    // cnt_pts_id.first(int), cnt_pts_id.second.first(cv::Point2f) 와 같이 불러올 수 있음.
    // cnt_pts_id.first : track_cnt[i] (feature 추적 횟수) --> int
    // cnt_pts_id.second.first : cur_pts[i] (feature location) --> cv::Point2f
    // cnt_pts_id.second.second : ids[i] (feature id) --> int

    for (unsigned int i = 0; i < cur_pts.size(); i++)       // data type 이 unsigned int 이기 때문에 i 는 양의 값만을 가지며, 대신 표현할 수 있는 범위가 int 의 2배이다.
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));

    // sort 함수를 통해 cnt_pts_id 를 내림차순 정렬한다. (track_cnt 기준)
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;       // track_cnt 비교
         });

    cur_pts.clear();   // cur_pts 값을 비운다. ( 이 값들은 지금 cnt_pts_id.second.first 에 저장되어 있는 상태 )
    ids.clear();       // ids 값을 비운다. ( 이 값들은 지금 cnt_pts_id.second.second 에 저장되어 있는 상태 )
    track_cnt.clear(); // track_cnt 값을 비운다. ( 이 값들은 지금 cnt_pts_id.first 에 저장되어 있는 상태 )


    for (auto &it : cnt_pts_id)     // cur_pts, ids, track_cnt 재구성. (조건에 해당하는 좌표들로)
    {
        if (mask.at<uchar>(it.second.first) == 255)     // cur_pts 에 해당하는 좌표의 픽셀 값이 255 인 경우
        // it.second.first 는 cv::Point2f cur_pts 를 의미함.
        {
            cur_pts.push_back(it.second.first);         // cur_pts(값을 비운 상태)를 재구성한다. (조건에 해당할 경우)
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);     // 해당하는 좌표를 이미지 프레임 위에 표시한다.
        }
    }
}

// cv::Point2f type 의 (pt1.x, pt1.y), (pt2.x, pt2.y) distance 를 유클리드 거리로 구하는 함수.
double FeatureTracker::distance(cv::Point2f &pt1, cv::Point2f &pt2)
{
    //printf("pt1: %f %f pt2: %f %f\n", pt1.x, pt1.y, pt2.x, pt2.y);
    double dx = pt1.x - pt2.x;
    double dy = pt1.y - pt2.y;
    return sqrt(dx * dx + dy * dy);
}

// SIVVON -- 세부적인 parameter name 고쳐야 할 수 있음
void FeatureTracker::TrackorMatchwithSPnSG(
    vector<cv::Point2f> &prev_pts,
    vector<cv::Point2f> &cur_pts,
    vector<cv::Point2f> &prevXY,
    vector<cv::Point2f> &curXY,
    vector<float> &Confidence,
    vector<uchar> &status,
    const float Threshold
    )
{
    if ((prevXY.size()!=curXY.size()) || (prevXY.size()!=Confidence.size()) || (curXY.size()!=Confidence.size()))
    {
        std::cout << "prevXY size is " << prevXY.size() << '\n';
        std::cout << "curXY size is " << curXY.size() << '\n';
        std::cout << "p2cCOnfidence size is " << Confidence.size() << '\n';

        assert(false);
    }

    int idx;
    for (auto &it : prev_pts)
    {
        if(find(prevXY.begin(), prevXY.end(), it) != prevXY.end())
        {
            idx = find(prevXY.begin(), prevXY.end(), it) - prevXY.begin();
            if (Confidence[idx] > Threshold)
            {
                cur_pts.push_back(curXY[idx]);
                status.push_back(true);
            }
            else
            {
                cur_pts.push_back(cv::Point2f(0,0));
                status.push_back(false);
            }
        }
        else
        {
            cur_pts.push_back(cv::Point2f(0,0));
            status.push_back(false);
        }
    }

    if ((prev_pts.size()!=cur_pts.size()) || (prev_pts.size()!=status.size()) || (cur_pts.size()!=status.size()))
    {
        std::cout << "prev_pts size is " << prev_pts.size() << '\n';
        std::cout << "cur_pts size is " << cur_pts.size() << '\n';
        std::cout << "status size is " << status.size() << '\n';
        assert(false);
    }
}

// SIVVON -- 세부적인 parameter name 고쳐야 할 수 있음
void FeatureTracker::SuperPointsToTrack(
    vector<cv::Point2f> &n_pts,
    vector<pair<float, cv::Point2f>> &curSuperPointFeature,
    cv::Mat mask,
    float KpThreshold,
    int n_max_cnt
    )
{
    n_pts.clear();
    sort(curSuperPointFeature.begin(), curSuperPointFeature.end(), [](const pair<float, cv::Point2f> &a, const pair<float, cv::Point2f> &b)
        {
            return a.first > b.first;
        });
    int cnt = 0;
    for (auto &it : curSuperPointFeature)
    {
        float confidence = it.first;
        cv::Point2f curXY = it.second;
        if ((mask.at<uchar>(curXY) == 255) && (confidence > KpThreshold)) 
        {
            n_pts.push_back(curXY);
        }
        cnt += 1;
        if (cnt > n_max_cnt)
            break;
    }
    // cout << "n_pts size in SuperPointsToTrack :" << n_pts.size() << '\n';
}


// 이 부분은 stereo 가 아님. mono 일 때임. 
// estimator.cpp 에서 이 함수 불러올 때가 mono 일 때임. --> 근데 왜 img1 이 있지?? 잘 모르겠음. 알아보기
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;                     // tic_toc.h
    cur_time = _cur_time;           // iamge frame rosbag time (input1)
    cur_img = _img;                 // cam0(left) image (input2)
    row = cur_img.rows;             // cam0(left) image row size
    col = cur_img.cols;             // cam0(left) image column size
    cv::Mat rightImg = _img1;       // cam1(right) image (input3)
    /*
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(cur_img, cur_img);
        if(!rightImg.empty())
            clahe->apply(rightImg, rightImg);
    }
    */
    cur_pts.clear();                // cur_pts (현재 이미지 프레임의 feature points) 값 비워서 초기화

    if (prev_pts.size() > 0)        // previous frame 이 존재하며, feature points 가 존재하는 경우
    {
        TicToc t_o;
        vector<uchar> status;       // uchar type (True or False) 의 status 벡터 (cur_pts 의 상태 정보를 나타냄.)
        vector<float> err;          // float type 의 error 벡터 (optical flow 함수 출력으로 채워질 것.)
        if(hasPrediction)           // hasPrediction == True 인 경우 --> FeatureTracker::setPrediction 에서만 True 로 setting 됨.
        {
            cur_pts = predict_pts;  // 현재 이미지 프레임의 feature points 를 predict_pts 에 할당한다.

            // optical flow 계산 (이전 프레임과 현재 프레임간 계산)
            /*
            prev_img, cur_img : 이전 프레임과 현제 프레임. 8비트 입력 영상.
            prev_pts : 이전 프레임에서 추적할 점들
            cur_pts : ( 출력 ) prev_pts feature points 가 이동한 (현재 프레임의) 좌표. prevPts가 어디로 이동했는지에 대한 정보를 저장. 입력으로는 None을 준다.
            status : ( 출력 ) 점들의 매칭 상태. numpy.ndarray. shape=(N, 1), dtype=np.uint8. i번째 원소가 1이면 prevPts의 i번째 점이 nextPts의 i번째 점으로 이동.
            err : ( 출력 ) 결과 오차 정보. numpy.ndarray. shape=(N, 1), dtype=np.float32.
            cv::Size(21, 21) : winSize. 각 피라미드 레벨에서 검색할 윈도우 크기. 기본값은 (21, 21).
            1 :  maxLevel == 최대 피라미드 레벨. 0이면 피라미드 사용 안 함. 기본값은 3.
            cv::TermCriteria : criteria. (반복 알고리즘의) 종료 기준
            */
            /*
            status 
            --> ndarray, (N,1), unit8 형태. 
            --> N은 이전 점에서 추적할 좌표 개수를 의미. 
            --> 추적하다가 점이 갑자기 사라질 수 있고, 알고리즘이 못찾아서 놓칠수 있음.
            --> 추적이 잘 됐는지 아닌지 알려주는 플래그 값이 저장되어 있는 행렬.
            --> 원소 값이 1 이면 추적이 잘 되었고 0 이면 추적을 잘 못한 것.
            */
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            // 수정 필요할까? --> 수정한다면, previous point 와 current point 가 모두 미리 구해진 상태일 때 구하는 함수로 바꾸어야 할 것!
            // 또한 status 도 고려해야 할 것.

            int succ_num = 0;                           // status 배열 중 1 (optical flow 추적 성공) 횟수 
            for (size_t i = 0; i < status.size(); i++)   
            {
                if (status[i])                          // status[i] == 1
                    succ_num++;
            }
            if (succ_num < 10)                          // 전후 프레임 optical flow 추적 성공 횟수가 3보다 작을 때.
                // 아래 함수가 아까와 달라진 것 : 피라미드 maxLevel 을 3 으로 지정함.
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else                        // hasPrediction 이 false 인 경우
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        // reverse check
        // 이번에는 아까 optical flow 의 출력으로 구한 cur_pts 를 입력으로 한다.
        // ( 출력 ) prev_pts : (아까 출력으로 구한) cur_pts 를 입력으로 넣고 optical flow 로 prev_pts 를 구한다.
        if(FLOW_BACK)
        {
            vector<uchar> reverse_status;                   // 검산(출력으로 얻은 cur_pts 를 활용해 다시 구해낸 status)
            vector<cv::Point2f> reverse_pts = prev_pts;     // ?? 여기서는 reverse_pts 가 비지 않네? 뭐지. 확인 필요!! 중요
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            //cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3); 
            for(size_t i = 0; i < status.size(); i++)       // status 를 기준으로 반복
            {
                // 검산 --> 검산하려면 기존에 계산한 값(status)과 역으로 계산한 값(reverse_status)을 비교해야 한다.
                // 검산 후 통과한 것들만 1 로 남긴다. (status 갱신)
                /*
                ( 조건 )
                status[i] == reverse_status[i] == 1 이면서, 
                prev_pts(원래 입력으로 준 prev_pts)와 reverse_pts(검산으로 얻은 prev_pts) 가 거의 일치할 경우 (유클리드거리 <= 0.5)
                */ 
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;      // status 갱신
                }
                else
                    status[i] = 0;      // status 갱신
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)       // cur_pts 를 기준으로 한다. --> status 도 어차피 각 좌표 매칭상태 정보이므로 size 가 같아야 한다.
        {
            if (status[i] && !inBorder(cur_pts[i]))         // status 가 1 임에도 cur_pts 가 이미지 테두리 안에 없으면 그냥 status 0 으로 변경해준다.
                status[i] = 0;            
        }
        
        // status 1 인 points 만 남도록 prev_pts, cur_pts, ids, track_cnt 재구성
        reduceVector(prev_pts, status);     // vector 에 포함되는 요소들 중 status(input2) 가 1 인 것들만 남도록 변환하는 함수. (이 코드 위쪽에서 선언된 함수임.)
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());      // optical flow 계산(및 검산) 에 소요된 총 시간
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)       // track_cnt (추적 성공 횟수) 만큼 n 이 증가함.
    {
        n++;
    }

    if (1)      // 무조건 실행됨.
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();      // FeatureTracker::setMask()
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());         // MAX_CNT : 200 으로 지정됨.
        if (n_max_cnt > 0)          // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적을 때
        {
            if(mask.empty())        // mask == (현재 이미지 프레임) --> setMask() 에서 지정됨.
            {
                cout << "mask is empty " << endl;
            }
            if (mask.type() != CV_8UC1)
            {
                cout << "mask type wrong " << endl;
            }
            
            /*
            cur_img : cam0(left) image --> 입력 이미지
            n_pts : ( 출력 ) 검출된 코너를 담을 벡터. ex) vector<Point2f>
            MAX_CNT - cur_pts.size() : 코너 최댓값. 검출할 최대 코너의 수를 제한한다. 코너 최댓값보다 낮은 개수만 반환. --> 이때 궁금한 것 : 좋은 feature 부터 반환하는 것인지? 확인 필요
            0.01 : qualityLevel. 코너라고 판단하기 위한 기준이 되는 최소의 값. (Threshold) 
            */
            /*
            qualityLevel
            --> 최고의 minimal eigenvalue를 가지는 코너의 quality에 곱해진다.
            --> 그 값보다 작은 quality를 갖는 코너는 코너라고 판단하지 않는다.
            --> 최고의 minimal eigenvalue=1500 이고, qualityLevel= 0.01 이면, quality가 15보다 작은 코너는 무시된다.
            */
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask); 
            // 여기서 새로 찾은 점들은 optical flow 와는 관계없는 게 맞는지?? 확인 필요. --> 예시를 들어서 생각해볼 것 (몇 번 도는 경우를 가정해서.)
        }
        else                    
        // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적지 않을 때 --> 많을 수도 있나? 그건 모르겠음. (위 optical flow 부분에서 확인)
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());        // 확인 필요 : 혹시 이 tic toc 부분 때문에 device 너무 빨리 움직인다는 경고가 뜨는 것인지?? 확인 필요.

        for (auto &p : n_pts)               // n_pts(goodFeaturesToTrack 으로 새로 찾은 점들) 
        // --> 이때 궁금한 것 : 그냥 n_pts 내부의 것을 다 넣는데, 그럼 기존 optical flow 로 찾은 점들과 중복되는 좌표가 있으면 어떻게 되는 것인지? 확인 필요
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);          // feature id를 의미
            track_cnt.push_back(1);
        }
        //printf("feature cnt after add %d\n", (int)ids.size());
    }
    
    cur_un_pts   = undistortedPts(cur_pts, m_camera[0]);    // cur_pts : 현재 프레임의 좌표 (옵티컬 플로우로 찾은 것 + goodFeaturesToTrack 으로 추가로 찾은 것)

    // cf. ptsVelocity --> FeatureTracker 의 멤버 함수. 해당 부분 확인 후 보충 필요.
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map); // ?? 2D velocity에 대한 계산


    //=======================================================================================================================
    //=== Stereo Matching Module=============================================================================================
    //=======================================================================================================================
    if(!_img1.empty() && stereo_cam)        
    // stereo cam 으로 설정되어 있으면서 _img1(cam1, right) 이 비어있지 않은 경우 --> 그럼 스테레오일 때도 돌아가는 건가? --> 아님. 
    // --> estimator.cpp 에서 이 함수 부르는 부분 보면 모노일 때만 이 부분 돌아감. 즉, 스테레오일때는 안 돌아감. 하지만 혹시나, 돌아가는지 print 등으로 확인 필요.
    {
        ids_right.clear();
        cur_right_pts.clear();
        cur_un_right_pts.clear();
        right_pts_velocity.clear();
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3);
            // reverse check cur right ---- cur left
            if(FLOW_BACK)                   // FLOW_BACK == 1 로 세팅됨.    
            {
                cv::calcOpticalFlowPyrLK(rightImg, cur_img, cur_right_pts, reverseLeftPts, statusRightLeft, err, cv::Size(21, 21), 3);
                for(size_t i = 0; i < status.size(); i++)
                {
                    if(status[i] && statusRightLeft[i] && inBorder(cur_right_pts[i]) && distance(cur_pts[i], reverseLeftPts[i]) <= 0.5)
                        status[i] = 1;
                    else
                        status[i] = 0;
                }
            }

            ids_right = ids;
            reduceVector(cur_right_pts, status);
            reduceVector(ids_right, status);
            // only keep left-right pts
            /*
            reduceVector(cur_pts, status);
            reduceVector(ids, status);
            reduceVector(track_cnt, status);
            reduceVector(cur_un_pts, status);
            reduceVector(pts_velocity, status);
            */
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            //cur_un_right_pts = undistortedPtsWidthOutDistortion(cur_right_pts);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }        
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);


    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); i++)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
    }

    //printf("feature track whole time %f\n", t_r.toc());
    return featureFrame;
}

// sivvon
// stereo cam 인 경우
// estimator.cpp 에서 이 함수가 돌 때가 stereo 인 경우임.
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImageBlockMatching(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1, int _seq)
{
    TicToc t_r;                     // tic_toc.h
    cur_time = _cur_time;           // iamge frame rosbag time (input1)
    cur_img = _img;                 // cam0(left) image (input2)
    row = cur_img.rows;             // cam0(left) image row size
    col = cur_img.cols;             // cam0(left) image column size
    cv::Mat rightImg = _img1;       // cam1(right) image (input3)
    
    cur_pts.clear();                // cur_pts (현재 이미지 프레임의 feature points) 값 비워서 초기화

    // SIVVON for tracking and matching with SuperPoint+SuperGlue
    vector<cv::Point2f> prevXY, curXY, curRXY;
    // vector<float> curKeyPointConfidence;
    vector<float> prev2curTrackConfidence;
    vector<float> curL2RstereoMatchConfidence;

    vector<pair<float, cv::Point2f>> curSuperPointFeature;                     // SIVVON, curKeyPointConfidence, curXY
    // vector<pair<float, pair<cv::Point2f, cv::Point2f>>> prev_cur_track_data;  // SIVVON, prev2curTrackConfidence, prevXY, curXY
    // vector<pair<float, pair<cv::Point2f, cv::Point2f>>> cur_L2R_match_data;   // SIVVON, curL2RstereoMatchConfidence, curXY, curRXY

    // sivvon 일반, euroc, catalonix
    // std::string seq_str = to_string(_seq);
    // std::cout << seq_str << '\n';
    // std::string SPSGdir = SP_SG_DATA_DIR + "/now_" + seq_str + ".txt";

    // sivvon kitti
    std::ostringstream st;
    st << std::setfill('0') << std::setw(6) << _seq;
    std::cout << st.str() << '\n';
    std::string SPSGdir = SP_SG_DATA_DIR + "/now_" + st.str() + ".txt";
    
    std::ifstream SPSGdata(SPSGdir);
    std::string line, buffer;

    if (SPSGdata.is_open())
    {
        while(getline(SPSGdata, line, '\n'))
            {
                std::vector<float> row;
                std::istringstream iss(line);
                float value;
                // std::cout<<line<<'\n';
                while(iss >> value){
                    row.push_back(value);
                    // std::cout<<value<<' ';
                }
                // std::cout<<'\n';
                // vect.push_back(row);
                // curX = row[2];
                // curY = row[3];
                // std::vector<float> curVec{curX, curY};
                // curPts.push_back(curVec);

                // 이 부분 VINS Euroc 기준으로 수정하기 SIVVON
                prevXY.push_back(cv::Point2f(row[0], row[1]));
                curXY.push_back(cv::Point2f(row[2], row[3]));
                curRXY.push_back(cv::Point2f(row[4], row[5]));
                // curKeyPointConfidence.push_back(row[6]);
                prev2curTrackConfidence.push_back(row[7]);
                curL2RstereoMatchConfidence.push_back(row[8]);

                curSuperPointFeature.push_back(make_pair(row[6], cv::Point2f(row[2], row[3])));
            }
    }
    else
    {
        std::cout << "----------------failed to open SPSGdata" << '\n';
    } //    SIVVON 여기서부터 하면 됨.


    if (prev_pts.size() > 0)        // previous frame 이 존재하며, feature points 가 존재하는 경우 (즉, 첫번째 프레임이 아닌 경우.)
    {
        TicToc t_o;
        vector<uchar> status;       // uchar type (True or False) 의 status 벡터 (cur_pts 의 상태 정보를 나타냄.)
        // vector<float> err;          // float type 의 error 벡터 (optical flow 함수 출력으로 채워질 것.)
        // if(hasPrediction)           // hasPrediction == True 인 경우 --> FeatureTracker::setPrediction 에서만 True 로 setting 됨.
        // // setPrediction 은 estimator.cpp 에서 실행되는 것을 확인함.
        // {
        //     cur_pts = predict_pts;  // predict_pts 를 현재 이미지 프레임의 feature points 에 할당한다.

        //     // optical flow 계산 (이전 프레임과 현재 프레임간 계산)
        //     /*
        //     prev_img, cur_img : 이전 프레임과 현제 프레임. 8비트 입력 영상.
        //     prev_pts : 이전 프레임에서 추적할 점들
        //     cur_pts : ( 출력 ) prev_pts feature points 가 이동한 (현재 프레임의) 좌표. prevPts가 어디로 이동했는지에 대한 정보를 저장. 입력으로는 None을 준다.
        //     status : ( 출력 ) 점들의 매칭 상태. numpy.ndarray. shape=(N, 1), dtype=np.uint8. i번째 원소가 1이면 prevPts의 i번째 점이 nextPts의 i번째 점으로 이동.
        //     err : ( 출력 ) 결과 오차 정보. numpy.ndarray. shape=(N, 1), dtype=np.float32.
        //     cv::Size(21, 21) : winSize. 각 피라미드 레벨에서 검색할 윈도우 크기. 기본값은 (21, 21).
        //     1 :  maxLevel == 최대 피라미드 레벨. 0이면 피라미드 사용 안 함. 기본값은 3.
        //     cv::TermCriteria : criteria. (반복 알고리즘의) 종료 기준
        //     */
        //     /*
        //     status 
        //     --> ndarray, (N,1), unit8 형태. 
        //     --> N은 이전 점에서 추적할 좌표 개수를 의미. 
        //     --> 추적하다가 점이 갑자기 사라질 수 있고, 알고리즘이 못찾아서 놓칠수 있음.
        //     --> 추적이 잘 됐는지 아닌지 알려주는 플래그 값이 저장되어 있는 행렬.
        //     --> 원소 값이 1 이면 추적이 잘 되었고 0 이면 추적을 잘 못한 것.
        //     */
        //     cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
        //     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
        //     // 수정 필요할까? --> 수정한다면, previous point 와 current point 가 모두 미리 구해진 상태일 때 구하는 함수로 바꾸어야 할 것!
        //     // 또한 status 도 고려해야 할 것.sd
            
        //     int succ_num = 0;                           // status 배열 중 1 (optical flow 추적 성공) 횟수
        //     for (size_t i = 0; i < status.size(); i++)
        //     {
        //         if (status[i])                          // status[i] > 0
        //             succ_num++;
        //     }
        //     if (succ_num < 10)                          // 전후 프레임 optical flow 추적 성공 횟수가 3보다 작을 때.
        //         // 아래 함수가 아까와 달라진 것 : 피라미드 maxLevel 을 3 으로 지정함.
        //         cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // }
        // else                        // hasPrediction 이 false 인 경우
        //     cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);

        // SIVVON optical flow 대체
        TrackorMatchwithSPnSG(prev_pts, cur_pts, prevXY, curXY, prev2curTrackConfidence, status, TRACK_THRESHOLD);
        
        // reverse check
        // 이번에는 아까 optical flow 의 출력으로 구한 cur_pts 를 입력으로 한다.
        // ( 출력 ) prev_pts : (아까 출력으로 구한) cur_pts 를 입력으로 넣고 optical flow 로 prev_pts 를 구한다.
        // if(FLOW_BACK)               // FLOW_BACK == flow_back == true (기본적으로 true 로 세팅된 파라미터)
        // {
        //     vector<uchar> reverse_status;                   // 검산(출력으로 얻은 cur_pts 를 활용해 다시 구해낸 status)
        //     vector<cv::Point2f> reverse_pts = prev_pts;     
        //     // ?? 여기서는 reverse_pts 가 비지 않네? 뭐지. 꼭 확인!! 중요 (출력이 비지 않아도 되는지)
        //     // --> 확인 결과 reverse_pts 는 InputOutputArray 이다. (opencv 홈페이지에서 확인하였다.) --> Input과 Output 의 기능을 동시에 수행.
        //     // 그러나 앞에서 확인한 결과 input 일 때 None 이어도 되는 듯하다. (재확인 필요)
        //     cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
        //     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);            
            
        //     for(size_t i = 0; i < status.size(); i++)       // status 를 기준으로 반복
        //     {
        //         // 검산 --> 검산하려면 기존에 계산한 값(status)과 역으로 계산한 값(reverse_status)을 비교해야 한다.
        //         // 검산 후 통과한 것들만 1 로 남긴다. (status 갱신)
        //         /*
        //         ( 조건 )
        //         status[i] == reverse_status[i] == 1 이면서, 
        //         prev_pts(원래 입력으로 준 prev_pts)와 reverse_pts(검산으로 얻은 prev_pts) 가 거의 일치할 경우 (유클리드거리 <= 0.5)
        //         */ 
        //         if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
        //         {
        //             status[i] = 1;      // status 갱신
        //         }
        //         else
        //             status[i] = 0;      // status 갱신
        //     }
        // }
        
        // 이건 일단 두기. 아마 optical flow 결과로 음수 나오고 이런 것 때문에 있는 것 같긴 함.
        // SIVVON 나중에 보고 지워도 잘 동작하면 걍 지우기
        for (int i = 0; i < int(cur_pts.size()); i++)       
        // cur_pts 를 기준으로 한다. --> status 도 어차피 각 좌표 매칭상태 정보이므로 size 가 cur_pts 와 같아야 한다.
        {
            if (status[i] && !inBorder(cur_pts[i]))         // status 가 1 임에도 cur_pts 가 이미지 테두리 안에 없으면 그냥 status 0 으로 변경해준다.
                status[i] = 0;            
        }

        // status 1 인 points 만 남도록 prev_pts, cur_pts, ids, track_cnt 재구성       
        reduceVector(prev_pts, status);     // vector 에 포함되는 요소들 중 status(input2) 가 1 인 것들만 남도록 변환하는 함수. (이 코드 위쪽에서 선언된 함수임.)
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());      // optical flow 계산(및 검산) 에 소요된 총 시간
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)       // track_cnt (추적 성공 횟수) 만큼 n 이 증가함.
        n++;

    if (1)      // 무조건 실행됨. --> 이 부분이 goodFeaturesToTrack part 인데 대체할 것
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();      // FeatureTracker::setMask()
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());         // MAX_CNT : 200 으로 지정됨. (궁금하면 이 코드 파일 말고 전체 파일에 대해 검색해볼 것.)
        if (n_max_cnt > 0)          // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적을 때
        {
            if(mask.empty())        // mask == (현재 이미지 프레임) --> setMask() 에서 지정됨. 8줄쯤 위에서 setMask() 실행함.
            {
                cout << "mask is empty " << endl;
            }    
            if (mask.type() != CV_8UC1)
            {
                cout << "mask type wrong " << endl;
            }
            
            /*
            cur_img : cam0(left) image --> 입력 이미지
            n_pts : ( 출력 ) 검출된 코너를 담을 벡터. ex) vector<Point2f>
            MAX_CNT - cur_pts.size() : 코너 최댓값. 검출할 최대 코너의 수를 제한한다. 코너 최댓값보다 낮은 개수만 반환. --> 이때 궁금한 것 : 좋은 feature 부터 반환하는 것인지? 확인 필요
            0.01 : [ qualityLevel. ] 코너라고 판단하기 위한 기준이 되는 최소의 값. (Threshold) 

            [ qualityLevel ]
            --> 최고의 minimal eigenvalue를 가지는 코너의 quality에 곱해진다.
            --> 그 값보다 작은 quality를 갖는 코너는 코너라고 판단하지 않는다.
            --> 최고의 minimal eigenvalue=1500 이고, qualityLevel= 0.01 이면, quality가 15보다 작은 코너는 무시된다.
            */
            // cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
            // ROS_INFO("what the hell");

            //SIVVON -- goodFeaturesToTrack 대체
            SuperPointsToTrack(n_pts, curSuperPointFeature, mask, KP_THRESHOLD, n_max_cnt);  

        }
        else
        // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적지 않을 때 --> 많을 수도 있나? 그건 모르겠음. 
        // 위 optical flow 부분에서 확인 필요. --> 개수로 찾지 않는 듯함.
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());        // 확인 필요 : 혹시 이 tic toc 부분 때문에 device 너무 빨리 움직인다는 경고가 뜨는 것인지?? 확인 필요.

        for (auto &p : n_pts)               // n_pts(goodFeaturesToTrack 으로 새로 찾은 점들) 
        // --> 이때 궁금한 것 : 그냥 n_pts 내부의 것을 다 넣는데, 그럼 기존 optical flow 로 찾은 점들과 중복되는 좌표가 있으면 어떻게 되는 것인지? 확인 필요
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);          // feature id를 의미? 확인 필요.
            track_cnt.push_back(1);
        }
    }
    
    cur_un_pts   = undistortedPts(cur_pts, m_camera[0]);    // cur_pts : 현재 프레임의 좌표 (옵티컬 플로우로 찾은 것 + goodFeaturesToTrack 으로 추가로 찾은 것)
    cout << "cur_un_pts size in SuperPointsToTrack :" << cur_un_pts.size() << '\n';

    // cf. FeatureTracker::ptsVelocity() --> FeatureTracker 의 멤버 함수.  
    // pts_velocity (x축 방향 속도, y축 방향 속도 데이터를 cv::Point2f 형태의 쌍으로 가짐.)
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    /*=========================================================================================================================
    ======================================= 여기서부터 스테레오 블록 매칭 구간입니다. =================================================
    ===========================================================================================================================*/
    if(!_img1.empty() && stereo_cam)
    {

        ids_right.clear();                  // ids_right : right(cam1) frame image id vector --> 비우기
        cur_right_pts.clear();              // cur_right_pts : right(cam1) frame image feature points vector --> 비우기
        cur_un_right_pts.clear();           // cur_un_right_pts : right(cam1) frame image undistorted points vector --> 비우기
        right_pts_velocity.clear();         // right_pts_velocity : cam1(right) image frame feature 를 통해 추출한 x 축 방향, y 축 방향 속도
        /* 궁금한 점 : 그럼 z 축 방향은 뭐로 알 수 있는 거임...? 인터불고 돌렸을 때 z축 이상하게 나온 이유가 뭔데...???
        --> 확인 결과 : z 축 계산 자체를 (x,y) 기반으로 정규화 후 벡터 거리 구하는 공식을 이용해서 하는 것으로 예상. 즉, x,y 가 이상하게 나오면 z 도 이상하게 나온 것 같음.
        --> 추가 확인 필요 사항 : 정규화를 하는 기준은 무엇인지?? 등등..
        */
        cur_un_right_pts_map.clear();

        
        if(!cur_pts.empty())                // 현재 프레임이 존재할 경우
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            // vector<uchar> status, statusRightLeft;
            vector<uchar> status;
            // vector<float> err;

            // stereo Block Matching : cur left ---- cur right
            // stereoBlockMatching.cpp 참고
            // calcStereoMatchingGoodFeature2Track(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3, mStereoKernelShiftY);
            TrackorMatchwithSPnSG(cur_pts, cur_right_pts, curXY, curRXY, curL2RstereoMatchConfidence, status, MATCH_THRESHOLD);
            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);      // cam1(right) undistorted points

            // FeatureTracker::epipolarConstrantOutlierRemover --> epipolar geometry 를 만족하지 않는 feature points status 를 0 으로 만든다.
            epipolarConstrantOutlierRemover(cur_un_pts, cur_un_right_pts, status, essentialMatrix, cur_pts);
            
            ids_right = ids;                        // cam1(right) feature points' id 를 cam0(left) feature points' id 와 같다고 한다.
            reduceVector(cur_right_pts, status);    // cur_right_pts(cam1 현재 프레임 feature points) 벡터 요소 중 status 가 1인 것만 남기고 나머지는 제거한다.
            reduceVector(ids_right, status);

            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            cout << "cur_un_right_pts size in SuperPointsToTrack :" << cur_un_right_pts.size() << '\n';
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }        
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);


    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        // ROS_INFO("feature1 : %d %d",camera_id, velocity_x);
    }

    // FeatureFrame size 프린트(방법 찾기)

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); ++i)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            // ROS_INFO("feature2 : %d %d",camera_id, velocity_x);
        }
    }
    
    // ids, ids_right size 각각 프린트(혹은 더한 것)
    // featureFrame size 프린트(방법 찾기)
    
    return featureFrame;
}

// vins original trackImageBlockMatching
// stereo cam 인 경우
// estimator.cpp 에서 이 함수가 돌 때가 stereo 인 경우임.
map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImageBlockMatching_origin(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    TicToc t_r;                     // tic_toc.h
    cur_time = _cur_time;           // iamge frame rosbag time (input1)
    cur_img = _img;                 // cam0(left) image (input2)
    row = cur_img.rows;             // cam0(left) image row size
    col = cur_img.cols;             // cam0(left) image column size
    cv::Mat rightImg = _img1;       // cam1(right) image (input3)
    
    cur_pts.clear();                // cur_pts (현재 이미지 프레임의 feature points) 값 비워서 초기화

    if (prev_pts.size() > 0)        // previous frame 이 존재하며, feature points 가 존재하는 경우 (즉, 첫번째 프레임이 아닌 경우.)
    {
        TicToc t_o;
        vector<uchar> status;       // uchar type (True or False) 의 status 벡터 (cur_pts 의 상태 정보를 나타냄.)
        vector<float> err;          // float type 의 error 벡터 (optical flow 함수 출력으로 채워질 것.)
        if(hasPrediction)           // hasPrediction == True 인 경우 --> FeatureTracker::setPrediction 에서만 True 로 setting 됨.
        // setPrediction 은 estimator.cpp 에서 실행되는 것을 확인함.
        {
            cur_pts = predict_pts;  // predict_pts 를 현재 이미지 프레임의 feature points 에 할당한다.

            // optical flow 계산 (이전 프레임과 현재 프레임간 계산)
            /*
            prev_img, cur_img : 이전 프레임과 현제 프레임. 8비트 입력 영상.
            prev_pts : 이전 프레임에서 추적할 점들
            cur_pts : ( 출력 ) prev_pts feature points 가 이동한 (현재 프레임의) 좌표. prevPts가 어디로 이동했는지에 대한 정보를 저장. 입력으로는 None을 준다.
            status : ( 출력 ) 점들의 매칭 상태. numpy.ndarray. shape=(N, 1), dtype=np.uint8. i번째 원소가 1이면 prevPts의 i번째 점이 nextPts의 i번째 점으로 이동.
            err : ( 출력 ) 결과 오차 정보. numpy.ndarray. shape=(N, 1), dtype=np.float32.
            cv::Size(21, 21) : winSize. 각 피라미드 레벨에서 검색할 윈도우 크기. 기본값은 (21, 21).
            1 :  maxLevel == 최대 피라미드 레벨. 0이면 피라미드 사용 안 함. 기본값은 3.
            cv::TermCriteria : criteria. (반복 알고리즘의) 종료 기준
            */
            /*
            status 
            --> ndarray, (N,1), unit8 형태. 
            --> N은 이전 점에서 추적할 좌표 개수를 의미. 
            --> 추적하다가 점이 갑자기 사라질 수 있고, 알고리즘이 못찾아서 놓칠수 있음.
            --> 추적이 잘 됐는지 아닌지 알려주는 플래그 값이 저장되어 있는 행렬.
            --> 원소 값이 1 이면 추적이 잘 되었고 0 이면 추적을 잘 못한 것.
            */
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);
            // 수정 필요할까? --> 수정한다면, previous point 와 current point 가 모두 미리 구해진 상태일 때 구하는 함수로 바꾸어야 할 것!
            // 또한 status 도 고려해야 할 것.sd
            
            int succ_num = 0;                           // status 배열 중 1 (optical flow 추적 성공) 횟수
            for (size_t i = 0; i < status.size(); i++)
            {
                if (status[i])                          // status[i] > 0
                    succ_num++;
            }
            if (succ_num < 10)                          // 전후 프레임 optical flow 추적 성공 횟수가 3보다 작을 때.
                // 아래 함수가 아까와 달라진 것 : 피라미드 maxLevel 을 3 으로 지정함.
                cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        }
        else                        // hasPrediction 이 false 인 경우
            cv::calcOpticalFlowPyrLK(prev_img, cur_img, prev_pts, cur_pts, status, err, cv::Size(21, 21), 3);
        // reverse check
        // 이번에는 아까 optical flow 의 출력으로 구한 cur_pts 를 입력으로 한다.
        // ( 출력 ) prev_pts : (아까 출력으로 구한) cur_pts 를 입력으로 넣고 optical flow 로 prev_pts 를 구한다.
        if(FLOW_BACK)               // FLOW_BACK == flow_back == true (기본적으로 true 로 세팅된 파라미터)
        {
            vector<uchar> reverse_status;                   // 검산(출력으로 얻은 cur_pts 를 활용해 다시 구해낸 status)
            vector<cv::Point2f> reverse_pts = prev_pts;     
            // ?? 여기서는 reverse_pts 가 비지 않네? 뭐지. 꼭 확인!! 중요 (출력이 비지 않아도 되는지)
            // --> 확인 결과 reverse_pts 는 InputOutputArray 이다. (opencv 홈페이지에서 확인하였다.) --> Input과 Output 의 기능을 동시에 수행.
            // 그러나 앞에서 확인한 결과 input 일 때 None 이어도 되는 듯하다. (재확인 필요)
            cv::calcOpticalFlowPyrLK(cur_img, prev_img, cur_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 1, 
            cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01), cv::OPTFLOW_USE_INITIAL_FLOW);            
            
            for(size_t i = 0; i < status.size(); i++)       // status 를 기준으로 반복
            {
                // 검산 --> 검산하려면 기존에 계산한 값(status)과 역으로 계산한 값(reverse_status)을 비교해야 한다.
                // 검산 후 통과한 것들만 1 로 남긴다. (status 갱신)
                /*
                ( 조건 )
                status[i] == reverse_status[i] == 1 이면서, 
                prev_pts(원래 입력으로 준 prev_pts)와 reverse_pts(검산으로 얻은 prev_pts) 가 거의 일치할 경우 (유클리드거리 <= 0.5)
                */ 
                if(status[i] && reverse_status[i] && distance(prev_pts[i], reverse_pts[i]) <= 0.5)
                {
                    status[i] = 1;      // status 갱신
                }
                else
                    status[i] = 0;      // status 갱신
            }
        }
        
        for (int i = 0; i < int(cur_pts.size()); i++)       
        // cur_pts 를 기준으로 한다. --> status 도 어차피 각 좌표 매칭상태 정보이므로 size 가 cur_pts 와 같아야 한다.
        {
            if (status[i] && !inBorder(cur_pts[i]))         // status 가 1 임에도 cur_pts 가 이미지 테두리 안에 없으면 그냥 status 0 으로 변경해준다.
                status[i] = 0;            
        }

        // status 1 인 points 만 남도록 prev_pts, cur_pts, ids, track_cnt 재구성       
        reduceVector(prev_pts, status);     // vector 에 포함되는 요소들 중 status(input2) 가 1 인 것들만 남도록 변환하는 함수. (이 코드 위쪽에서 선언된 함수임.)
        reduceVector(cur_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());      // optical flow 계산(및 검산) 에 소요된 총 시간
        //printf("track cnt %d\n", (int)ids.size());
    }

    for (auto &n : track_cnt)       // track_cnt (추적 성공 횟수) 만큼 n 이 증가함.
        n++;

    if (1)      // 무조건 실행됨.
    {
        //rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask();      // FeatureTracker::setMask()
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(cur_pts.size());         // MAX_CNT : 200 으로 지정됨. (궁금하면 이 코드 파일 말고 전체 파일에 대해 검색해볼 것.)
        if (n_max_cnt > 0)          // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적을 때
        {
            if(mask.empty())        // mask == (현재 이미지 프레임) --> setMask() 에서 지정됨. 8줄쯤 위에서 setMask() 실행함.
            {
                cout << "mask is empty " << endl;
            }    
            if (mask.type() != CV_8UC1)
            {
                cout << "mask type wrong " << endl;
            }
            
            /*
            cur_img : cam0(left) image --> 입력 이미지
            n_pts : ( 출력 ) 검출된 코너를 담을 벡터. ex) vector<Point2f>
            MAX_CNT - cur_pts.size() : 코너 최댓값. 검출할 최대 코너의 수를 제한한다. 코너 최댓값보다 낮은 개수만 반환. --> 이때 궁금한 것 : 좋은 feature 부터 반환하는 것인지? 확인 필요
            0.01 : [ qualityLevel. ] 코너라고 판단하기 위한 기준이 되는 최소의 값. (Threshold) 

            [ qualityLevel ]
            --> 최고의 minimal eigenvalue를 가지는 코너의 quality에 곱해진다.
            --> 그 값보다 작은 quality를 갖는 코너는 코너라고 판단하지 않는다.
            --> 최고의 minimal eigenvalue=1500 이고, qualityLevel= 0.01 이면, quality가 15보다 작은 코너는 무시된다.
            */
            cv::goodFeaturesToTrack(cur_img, n_pts, MAX_CNT - cur_pts.size(), 0.01, MIN_DIST, mask);
            ROS_INFO("what the hell");
            // 여기서 새로 찾은 점들은 optical flow 와는 관계없는 게 맞는지?? 확인 필요. --> 예시를 들어서 생각해볼 것 (몇 번 도는 경우를 가정해서.)
        }
        else
        // MAX_CNT(200) 개수보다 cur_pts(현재 프레임 feature points 수) 가 적지 않을 때 --> 많을 수도 있나? 그건 모르겠음. 
        // 위 optical flow 부분에서 확인 필요. --> 개수로 찾지 않는 듯함.
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %f ms", t_t.toc());        // 확인 필요 : 혹시 이 tic toc 부분 때문에 device 너무 빨리 움직인다는 경고가 뜨는 것인지?? 확인 필요.

        for (auto &p : n_pts)               // n_pts(goodFeaturesToTrack 으로 새로 찾은 점들) 
        // --> 이때 궁금한 것 : 그냥 n_pts 내부의 것을 다 넣는데, 그럼 기존 optical flow 로 찾은 점들과 중복되는 좌표가 있으면 어떻게 되는 것인지? 확인 필요
        {
            cur_pts.push_back(p);
            ids.push_back(n_id++);          // feature id를 의미? 확인 필요.
            track_cnt.push_back(1);
        }
    }
    
    cur_un_pts   = undistortedPts(cur_pts, m_camera[0]);    // cur_pts : 현재 프레임의 좌표 (옵티컬 플로우로 찾은 것 + goodFeaturesToTrack 으로 추가로 찾은 것)

    // cf. FeatureTracker::ptsVelocity() --> FeatureTracker 의 멤버 함수.  
    // pts_velocity (x축 방향 속도, y축 방향 속도 데이터를 cv::Point2f 형태의 쌍으로 가짐.)
    pts_velocity = ptsVelocity(ids, cur_un_pts, cur_un_pts_map, prev_un_pts_map);

    /*=========================================================================================================================
    ======================================= 여기서부터 스테레오 블록 매칭 구간입니다. =================================================
    ===========================================================================================================================*/
    if(!_img1.empty() && stereo_cam)
    {
        ids_right.clear();                  // ids_right : right(cam1) frame image id vector --> 비우기
        cur_right_pts.clear();              // cur_right_pts : right(cam1) frame image feature points vector --> 비우기
        cur_un_right_pts.clear();           // cur_un_right_pts : right(cam1) frame image undistorted points vector --> 비우기
        right_pts_velocity.clear();         // right_pts_velocity : cam1(right) image frame feature 를 통해 추출한 x 축 방향, y 축 방향 속도
        /* 궁금한 점 : 그럼 z 축 방향은 뭐로 알 수 있는 거임...? 인터불고 돌렸을 때 z축 이상하게 나온 이유가 뭔데...???
        --> 확인 결과 : z 축 계산 자체를 (x,y) 기반으로 정규화 후 벡터 거리 구하는 공식을 이용해서 하는 것으로 예상. 즉, x,y 가 이상하게 나오면 z 도 이상하게 나온 것 같음.
        --> 추가 확인 필요 사항 : 정규화를 하는 기준은 무엇인지?? 등등..
        */
        cur_un_right_pts_map.clear();
        if(!cur_pts.empty())                // 현재 프레임이 존재할 경우
        {
            //printf("stereo image; track feature on right image\n");
            vector<cv::Point2f> reverseLeftPts;
            vector<uchar> status, statusRightLeft;
            vector<float> err;

            // stereo Block Matching : cur left ---- cur right
            // stereoBlockMatching.cpp 참고
            calcStereoMatchingGoodFeature2Track(cur_img, rightImg, cur_pts, cur_right_pts, status, err, cv::Size(21, 21), 3, mStereoKernelShiftY);

            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);      // cam1(right) undistorted points

            // FeatureTracker::epipolarConstrantOutlierRemover --> epipolar geometry 를 만족하지 않는 feature points status 를 0 으로 만든다.
            epipolarConstrantOutlierRemover(cur_un_pts, cur_un_right_pts, status, essentialMatrix, cur_pts);
            
            ids_right = ids;                        // cam1(right) feature points' id 를 cam0(left) feature points' id 와 같다고 한다.
            reduceVector(cur_right_pts, status);    // cur_right_pts(cam1 현재 프레임 feature points) 벡터 요소 중 status 가 1인 것만 남기고 나머지는 제거한다.
            reduceVector(ids_right, status);

            cur_un_right_pts = undistortedPts(cur_right_pts, m_camera[1]);
            right_pts_velocity = ptsVelocity(ids_right, cur_un_right_pts, cur_un_right_pts_map, prev_un_right_pts_map);
        }        
        prev_un_right_pts_map = cur_un_right_pts_map;
    }
    if(SHOW_TRACK)
        drawTrack(cur_img, rightImg, ids, cur_pts, cur_right_pts, prevLeftPtsMap);


    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    prev_un_pts_map = cur_un_pts_map;
    prev_time = cur_time;
    hasPrediction = false;

    prevLeftPtsMap.clear();
    for(size_t i = 0; i < cur_pts.size(); i++)
        prevLeftPtsMap[ids[i]] = cur_pts[i];

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int feature_id = ids[i];
        double x, y ,z;
        x = cur_un_pts[i].x;
        y = cur_un_pts[i].y;
        z = 1;
        double p_u, p_v;
        p_u = cur_pts[i].x;
        p_v = cur_pts[i].y;
        int camera_id = 0;
        double velocity_x, velocity_y;
        velocity_x = pts_velocity[i].x;
        velocity_y = pts_velocity[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        // ROS_INFO("feature1 : %d %d",camera_id, velocity_x);
    }

    // FeatureFrame size 프린트(방법 찾기)

    if (!_img1.empty() && stereo_cam)
    {
        for (size_t i = 0; i < ids_right.size(); ++i)
        {
            int feature_id = ids_right[i];
            double x, y ,z;
            x = cur_un_right_pts[i].x;
            y = cur_un_right_pts[i].y;
            z = 1;
            double p_u, p_v;
            p_u = cur_right_pts[i].x;
            p_v = cur_right_pts[i].y;
            int camera_id = 1;
            double velocity_x, velocity_y;
            velocity_x = right_pts_velocity[i].x;
            velocity_y = right_pts_velocity[i].y;

            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            ROS_INFO("feature2 : %d %d",camera_id, velocity_x);
        }
    }
    
    // ids, ids_right size 각각 프린트(혹은 더한 것)
    // featureFrame size 프린트(방법 찾기)
    
    return featureFrame;
}

void FeatureTracker::rejectWithF()
{
    if (cur_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_prev_pts(prev_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera[0]->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera[0]->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + col / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + row / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_prev_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, cur_pts.size(), 1.0 * cur_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

void FeatureTracker::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);        
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(row + 600, col + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < col; i++)
        for (int j = 0; j < row; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera[0]->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + col / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + row / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < row + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < col + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    // turn the following code on if you need
    // cv::imshow(name, undistortedImg);
    // cv::waitKey(0);
}

// (x,y) 2차원 벡터로 (x,y,z) 3차원 벡터 구성한 후 다시 2차원 벡터를 계산하여 보다 정확한 2차원 벡터를 얻는 함수. 수정 필요.
// pts(input1) : 2차원 feature point 좌표
// cam(input2) : 
vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{    
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;                      // 3차원 벡터 b?
        cam->liftProjective(a, b);              // b : a(x,y 값을 가지는 2차원 벡터)로 만든 3차원 벡터
        // liftProjective() 함수 확인 필요 --> 아마 (x,y,z) 벡터 길이를 1 로 두고 정규화해서 구하는 것 같긴 한데, 잘 모르겠음.. 재확인 필요.
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));     // z 축 상으로 왜곡된 값을 없애 줘서 왜곡되지 않은 2차원 벡터 (x,y) 값을 얻는다.
    }
    return un_pts;      // z축 방향으로의 왜곡을 보정한 2차원 벡터 (x,y)
}

// Right to Left _essentialMatrix
// epipolar geometry 를 만족하는지 여부를 검사하고, 만족하지 않으면 status 를 0 으로 지정하는 함수.
void FeatureTracker::epipolarConstrantOutlierRemover
(vector<cv::Point2f> &_unLeftpts, vector<cv::Point2f> &_unRightpts,
vector<uchar> &_status, const Eigen::Matrix3f &_essentialMatrix, vector<cv::Point2f> &_cur_pts) 
{    
    const float thresholdEpipolar = 0.0000001;
    const size_t dataSize = _status.size();
    // std::cerr << "_unLeftpts.size(): " << _unLeftpts.size() << std::endl;
    // std::cerr << "_unRightpts.size(): " << _unRightpts.size() << std::endl;
    // std::cerr << "_status.size(): " << _status.size() << std::endl;

    Eigen::MatrixXf unLeftPtsH(1,3);                // unLeftPtsH : cam0(Left) 프레임 영상에 존재하는 점 p = [ u v 1 ]
    Eigen::MatrixXf unRightPtsH(3,1);               // unRightPtsH : cam1(Right) 프레임 영상에 존재하는 점 p' = [ u' v' 1 ]T
    unLeftPtsH(0,2) = 0;                            // unLeftPtsH : cam0(Left) 위의 점 p 를 [ u v 0 ] 으로 초기화 (2번째 요소를 0 으로)
    unRightPtsH(2,0) = 0;                           // unRightPtsH : cam1(Right) 위의 점 p' 을 [ u v 0 ]T 로 초기화

    for(int i = 0; i < dataSize; ++i)               // dataSize == status size
    {
        if(_status[i])                              // _status[i] == true 
        {
            unLeftPtsH(0,0) = _unLeftpts[i].x;      // cam0(Left) feature point's x
            unLeftPtsH(0,1) = _unLeftpts[i].y;      // cam0(Left) feature point's y
            unRightPtsH(0,0) = _unRightpts[i].x;    // cam1(Right) feature point's x
            unRightPtsH(1,0) = _unRightpts[i].y;    // cam1(Right) featire point's y

            Eigen::MatrixXf costOfEssesnetialMatrix = unLeftPtsH*_essentialMatrix*unRightPtsH;      // epipolar geometry 에 따라 p'Ep=0 을 항상 만족해야 한다.

            if(std::abs(costOfEssesnetialMatrix(0,0)) > thresholdEpipolar)          // 근사하게라도 0을 만족해야 하는데, 그러지 못한 경우
            {
                _status[i] = 0;                                                     // epipolar geometry 를 만족하지 못하였음.
            }
            // else
            // {
            //     // std::cerr << "_Left_pts: " << _cur_pts[i] << std::endl;
            //     // std::cerr << "_unLeftpts: " << _unLeftpts[i] << std::endl;
            //     // std::cerr << "_unRightpts: " << _unRightpts[i] << std::endl;
            //     // std::cerr << "costOfEssesnetialMatrix: " << costOfEssesnetialMatrix << "\n\n" << std::endl;
            // }
        }
    }
};

vector<cv::Point2f> FeatureTracker::undistortedPts(vector<cv::KeyPoint> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].pt.x, pts[i].pt.y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        // std::cerr << "pts: " << a << ", un_pts: " << un_pts[un_pts.size()-1] << std::endl;
    }
    return un_pts;
}

// pts_velocity(x,y 축 방향 각각으로의 속도 data 를 저장하는 vector) 를 구하는 함수.
vector<cv::Point2f> FeatureTracker::ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
                                            map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts)
{
    vector<cv::Point2f> pts_velocity;                   // pts_velocity.x : v_x (x 축 방향의 velocity), pts_velocity.y : v_y (y 축 방향의 velocity)
    cur_id_pts.clear();                                 // cur_id_pts : ids[i], pts[i] 가 쌍을 이루는 map type data (아래 반복문에서 확인 가능)
    for (unsigned int i = 0; i < ids.size(); i++)       // ids vector size 만큼 반복 , i 는 양수만 가능 (unsigned int)
    {
        cur_id_pts.insert(make_pair(ids[i], pts[i]));
    }

    // caculate points velocity
    if (!prev_id_pts.empty())                                   // 이전 프레임의 points id들이 존재하는 경우
    {
        double dt = cur_time - prev_time;                       // dt : 시간 변화량 = 현재 프레임 시간 - 직전 프레임 시간
        
        for (unsigned int i = 0; i < pts.size(); i++)           // pts(feature point) size 만큼 반복
        {
            std::map<int, cv::Point2f>::iterator it;            // 반복자 it 선언.
            it = prev_id_pts.find(ids[i]);                      // it : prev_id_pts(ids[i], pts[i] 쌍 map) 에서 ids[i] 에 해당하는 key 값 찾아 할당.
            if (it != prev_id_pts.end())                        // it 가 prev_id_pts 의 마지막 key 값(ids)보다 이전 key 값(ids)일 때
            {
                double v_x = (pts[i].x - it->second.x) / dt;    
                // v_x : x 축 방향으로의 velocity --> pts[i].x (현재 프레임 feature x 좌표) - it->second.x (previous pts[i].x, 이전 프레임 feature x 좌표)
                double v_y = (pts[i].y - it->second.y) / dt;    // v_y : y 축 방향으로의 velocity
                pts_velocity.push_back(cv::Point2f(v_x, v_y));  // pts_velocity : x 축 방향의 velocity, y 축 방향의 velocity 로 구성한 cv::Point2f 타입의 vector
            }
            else
                pts_velocity.push_back(cv::Point2f(0, 0));      // prev_velocity 의 마지막에 반복자 it 가 도달할 경우 (0, 0) 넣음.
        }
    }
    else                                                        // 이전 프레임의 points id 들이 존재하지 않을 경우(이전 프레임이 존재하지 않거나, 이전 프레임에 feature 가 없거나 등)
    {
        for (unsigned int i = 0; i < pts.size(); i++)           
        {
            pts_velocity.push_back(cv::Point2f(0, 0));          // 속도 (velocity) 값이 존재하지 않는다.
        }
    }

    return pts_velocity;                        // pts velocity : x 축 방향의 velocity, y 축 방향의 velocity 로 구성한 cv::Point2f 타입의 vector
}

void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::Point2f> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    int rows = imLeft.rows;                                     // cam0(left image --> imLeft) row
    int cols = imLeft.cols;                                     // cam1(right image --> imRight) col
    if (!imRight.empty() && stereo_cam)                         // cam1(right) image frame 이 존재하면서 스테레오일때
        // cv::vconcat(imLeft, imRight, imTrack);
        cv::hconcat(imLeft, imRight, imTrack);                  // imTrack : cv::hconcat 함수를 이용해 imLeft, imRight 를 합친 결과 이미지 (가로로 합침.)
        
    else
        imTrack = imLeft.clone();                               // 합칠 cam1(right) image 가 없을 경우 imTrack 은 imLeft 와 같다고 한다.
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);         // 



    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            //rightPt.y += rows;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i], mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }
}


void FeatureTracker::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
                               vector<int> &curLeftIds,
                               vector<cv::KeyPoint> &curLeftPts, 
                               vector<cv::Point2f> &curRightPts,
                               map<int, cv::Point2f> &prevLeftPtsMap)
{
    //int rows = imLeft.rows;
    int cols = imLeft.cols;
    if (!imRight.empty() && stereo_cam)
        cv::hconcat(imLeft, imRight, imTrack);
    else
        imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, cv::COLOR_GRAY2RGB);

    for (size_t j = 0; j < curLeftPts.size(); j++)
    {
        double len = std::min(1.0, 1.0 * track_cnt[j] / 20);
        cv::circle(imTrack, curLeftPts[j].pt, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }
    if (!imRight.empty() && stereo_cam)
    {
        for (size_t i = 0; i < curRightPts.size(); i++)
        {
            cv::Point2f rightPt = curRightPts[i];
            rightPt.x += cols;
            cv::circle(imTrack, rightPt, 2, cv::Scalar(0, 255, 0), 2);
            //cv::Point2f leftPt = curLeftPtsTrackRight[i];
            //cv::line(imTrack, leftPt, rightPt, cv::Scalar(0, 255, 0), 1, 8, 0);
        }
    }
    
    map<int, cv::Point2f>::iterator mapIt;
    for (size_t i = 0; i < curLeftIds.size(); i++)
    {
        int id = curLeftIds[i];
        mapIt = prevLeftPtsMap.find(id);
        if(mapIt != prevLeftPtsMap.end())
        {
            cv::arrowedLine(imTrack, curLeftPts[i].pt, mapIt->second, cv::Scalar(0, 255, 0), 1, 8, 0, 0.2);
        }
    }

}


void FeatureTracker::setPrediction(map<int, Eigen::Vector3d> &predictPts)
{
    hasPrediction = true;           // hasPrediction 값 변경 (hasPrediction 의 default 는 false 로 세팅되어 있었음.)
    predict_pts.clear();            // predict_pts 값 비우기
    predict_pts_debug.clear();      
    map<int, Eigen::Vector3d>::iterator itPredict;          // 반복자 itPredict 선언
    for (size_t i = 0; i < ids.size(); i++)                 // ids : vector (feature points' id)
    {
        //printf("prevLeftId size %d prevLeftPts size %d\n",(int)prevLeftIds.size(), (int)prevLeftPts.size());
        int id = ids[i];                                    // id : ids vector 의 i 번째 요소
        itPredict = predictPts.find(id);                    // predictPts
        if (itPredict != predictPts.end())
        {
            Eigen::Vector2d tmp_uv;
            m_camera[0]->spaceToPlane(itPredict->second, tmp_uv);
            predict_pts.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
            predict_pts_debug.push_back(cv::Point2f(tmp_uv.x(), tmp_uv.y()));
        }
        else
            predict_pts.push_back(prev_pts[i]);
    }
}


void FeatureTracker::removeOutliers(set<int> &removePtsIds)
{
    std::set<int>::iterator itSet;
    vector<uchar> status;
    for (size_t i = 0; i < ids.size(); i++)
    {
        itSet = removePtsIds.find(ids[i]);
        if(itSet != removePtsIds.end())
            status.push_back(0);
        else
            status.push_back(1);
    }

    reduceVector(prev_pts, status);
    reduceVector(ids, status);
    reduceVector(track_cnt, status);
}


cv::Mat FeatureTracker::getTrackImage()
{
    return imTrack;
}
#ifndef __STEREO_VO_H
#define __STEREO_VO_H
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>


using namespace std;
using namespace Eigen;
using namespace cv;

class stereo_vo
{
private:
    int max_feat_cnt = 500;
    int min_feat_dist = 10;

public:

    stereo_vo(/* args */);
    ~stereo_vo();
};

stereo_vo::stereo_vo(/* args */)
{
}

stereo_vo::~stereo_vo()
{
}


#endif

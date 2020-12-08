#ifndef __STEREO_VO_H
#define __STEREO_VO_H
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace Eigen;
using namespace cv;

class stereo_vo
{
private:
    int max_feat_cnt = 500;
    int min_feat_dist = 10;

    //camera matrix
    Matrix3d K;
    double baseline;
public:

    stereo_vo(/* args */);
    ~stereo_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);
};

stereo_vo::stereo_vo(/* args */)
{
}

stereo_vo::~stereo_vo()
{
}

void stereo_vo::set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg)
{
    K = Matrix3d::Identity();
    K(0, 0) = msg->K[0];
    K(1, 1) = msg->K[4];
    K(0, 2) = msg->K[2];
    K(1, 2) = msg->K[5];
    baseline = msg->P[3] / msg->K[0];//
    cout << "K: " << K << endl;
    cout << "baseline: " << baseline << endl;
}

#endif

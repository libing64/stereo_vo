#ifndef __STEREO_VO_H
#define __STEREO_VO_H
#include <iostream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace Eigen;
using namespace cv;

Vector3d point2eigen(Point3f& p)
{

}

Point3f eigen2point(Vector3d& p)
{
    Point3f pp;
    pp.x = p(0);
    pp.y = p(1);
    pp.z = p(2);
    return pp;
}

class stereo_vo
{
private:
    int max_feat_cnt = 500;
    int min_feat_cnt = 100;
    int min_feat_dist = 10;
    int min_disparity = 2;
    int max_epipolar = 5;
    //camera matrix
    Matrix3d K;
    double baseline;
public:
    Quaterniond q;
    Vector3d t;

    int is_camera_info_init;
    vector<Point3f> feat3ds;
    vector<Point2f> feats;
    Mat masks;
    stereo_vo();
    ~stereo_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);
    void stereo_triangulate(Mat &left_img, Mat &right_img, vector<Point2f>& feats, vector<Point3f>& feat3ds);
    void update(Mat& left_img, Mat& right_img);
};

stereo_vo::stereo_vo()
{
    is_camera_info_init = false;
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
    baseline = fabs(msg->P[3] / msg->K[0]);
    cout << "K: " << K << endl;
    cout << "baseline: " << baseline << endl;
    is_camera_info_init = true;
}

void stereo_vo::update(Mat &left_img, Mat &right_img)
{
    if (feat3ds.empty() || feats.empty())
    {
        q = Quaterniond::Identity();
        t = Vector3d::Zero();

        int max_corners = max_feat_cnt;
        double quality_level = 0.01;
        double min_distance = min_feat_dist;
        int block_size = 3;
        bool use_harris = false;
        double k = 0.04;


        //feature detection
        cv::goodFeaturesToTrack(left_img, 
                                feats, 
                                max_corners, 
                                quality_level, 
                                min_distance, 
                                cv::Mat(), 
                                block_size, 
                                use_harris, k);
        cout << "good feature size: " << feats.size() << endl;
        stereo_triangulate(left_img, right_img, feats, feat3ds);
    } else 
    {

    }
}

void stereo_vo::stereo_triangulate(Mat &left_img, Mat &right_img, vector<Point2f> &feats, vector<Point3f> &feat3ds)
{
    //triangulation
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_right;
    cv::calcOpticalFlowPyrLK(left_img, right_img, feats, feats_right, status, err);

    double cx = K(0, 2);
    double cy = K(1, 2);
    double fx = K(0, 0);
    feat3ds.resize(feats.size());
    Point3f p;
    for (auto i = 0; i < feats.size(); i++)
    {
        if (status[i])
        {
            double dy = feats[i].y - feats_right[i].y;
            double dx = feats[i].x - feats_right[i].x;
            if (fabs(dx) > min_disparity && fabs(dy) < max_epipolar)
            {
                p.z = fx * baseline / dx;
                p.x = (feats[i].x - cx) / fx * p.z;
                p.y = (feats[i].y - cy) / fx * p.z;
                feat3ds[i] = p;
            } else 
            {
                p.x = 0;
                p.y = 0;
                p.z = 0;
                status[i] = 0;
                feat3ds[i] = p;
            }
        }
    }

}

#endif

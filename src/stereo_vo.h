#ifndef __STEREO_VO_H
#define __STEREO_VO_H
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
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
    int min_track_cnt = 50;
    int min_feat_dist = 10;
    int min_disparity = 2;
    int max_epipolar = 5;
    bool feat_vis_enable = true;
    //camera matrix
    Matrix3d K;
    double baseline;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    Mat keyframe_img;
public:
    Quaterniond q;
    Vector3d t;

    //state of keyframe
    Quaterniond qk;
    Vector3d tk;

    int is_camera_info_init;
    vector<Point3f> feat3ds;
    vector<Point2f> feats;
    Mat masks;
    stereo_vo();
    ~stereo_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);
    void stereo_triangulate(Mat &left_img, Mat &right_img, vector<Point2f>& feats, vector<Point3f>& feat3ds);

    void stereo_detect(Mat &left_img, Mat &right_img);
    void stereo_visualize(Mat &left_img, Mat &right_img, vector<Point2f>&left_feats, vector<Point2f>&right_feats);

    void update(Mat& left_img, Mat& right_img);
    void update_keyframe(Mat &left_img, Mat &right_img);
    int get_valid_feat_cnt(vector<uchar>& status);
    void visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar> &status);
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


    camera_matrix = Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0, 0) = msg->K[0];
    camera_matrix.at<double>(1, 1) = msg->K[4];
    camera_matrix.at<double>(0, 2) = msg->K[2];
    camera_matrix.at<double>(1, 2) = msg->K[5];

    dist_coeffs = Mat::zeros(5, 1, CV_64F);
    is_camera_info_init = true;
}

void stereo_vo::stereo_detect(Mat &left_img, Mat &right_img)
{
    Ptr<FeatureDetector> detector = cv::ORB::create();
    vector<KeyPoint> left_keypoints;
    detector->detect(left_img, left_keypoints);
    vector<Point2f> left_feats, right_feats;
    KeyPoint::convert(left_keypoints, left_feats);

    vector<uchar> status;
    vector<float> err;
    cv::calcOpticalFlowPyrLK(left_img, right_img, left_feats, right_feats, status, err);
    //visualize_features(left_img, left_feats, right_feats, status);

    double cx = K(0, 2);
    double cy = K(1, 2);
    double fx = K(0, 0);
    int count = std::count(status.begin(), status.end(), 1);
    feat3ds.resize(count);
    feats.resize(count);

    int j = 0;
    for (int i = 0; i < left_feats.size(); i++)
    {
        if (status[i])
        {
            Point3f p;
            double dy = left_feats[i].y - right_feats[i].y;
            double dx = left_feats[i].x - right_feats[i].x;
            if (fabs(dx) > min_disparity && fabs(dy) < max_epipolar)
            {
                p.z = fx * baseline / dx;
                p.x = (left_feats[i].x - cx) / fx * p.z;
                p.y = (left_feats[i].y - cy) / fx * p.z;
            }
            else
            {
                p.x = 0;
                p.y = 0;
                p.z = 0;
            }
            cout << "dxdy: " << dx << " " << dy << endl;
            cout << "p: " << p << endl;
            feat3ds[j] = p;
            feats[j] = left_feats[i];
            j++;
        }
    }
    //cout << "feat3ds: " << feat3ds << endl;
    left_img.copyTo(keyframe_img);
    stereo_visualize(left_img, right_img, left_feats, right_feats);
}

void stereo_vo::stereo_visualize(Mat &left_img, Mat &right_img, vector<Point2f> &left_feats, vector<Point2f> &right_feats)
{
    if (feat_vis_enable)
    {
        Mat merge_img = Mat(left_img.rows * 2, left_img.cols, CV_8UC3);
        if (left_img.channels() == 3)
        {
            left_img.copyTo(merge_img(Rect(0, 0, left_img.cols, left_img.rows)));
            right_img.copyTo(merge_img(Rect(0, left_img.rows, right_img.cols, right_img.rows)));
        }
        else
        {
            Mat left_img_color, right_img_color;
            cvtColor(left_img, left_img_color, cv::COLOR_GRAY2RGB);
            cvtColor(right_img, right_img_color, cv::COLOR_GRAY2RGB);
            left_img_color.copyTo(merge_img(Rect(0, 0, left_img.cols, left_img.rows)));
            right_img_color.copyTo(merge_img(Rect(0, left_img.rows, right_img.cols, right_img.rows)));
        }

        Scalar color = Scalar(0, 0, 255);
        Scalar left_color = Scalar(255, 0, 0);
        Scalar right_color = Scalar(0, 255, 0);
        Point2f delta(0, left_img.rows);
        for (int i = 0; i < feats.size(); i++)
        {
            //if (status[i])
            {
                line(merge_img, left_feats[i], right_feats[i] + delta, color, 1, 8);
                circle(merge_img, left_feats[i], 1, left_color);
                circle(merge_img, right_feats[i], 1, right_color);
            }
        }
        imshow("stereo match", merge_img);
        waitKey(2);
    }
}

void stereo_vo::update(Mat &left_img, Mat &right_img)
{
    if (feat3ds.empty() || feats.empty())
    {
        q = Quaterniond::Identity();
        t = Vector3d::Zero();

        stereo_detect(left_img, right_img);
    } else 
    {
        //feature tracking
        vector<uchar> status;
        vector<float> err;
        vector<Point2f> feats_next;
        cv::calcOpticalFlowPyrLK(keyframe_img, left_img, feats, feats_next, status, err);
        visualize_features(left_img, feats, feats_next, status);
        //waitKey(0);
        int valid_cnt = get_valid_feat_cnt(status);
        if (valid_cnt < min_track_cnt)//update keyframe
        {
            stereo_detect(left_img, right_img);
        } else 
        {
            vector<Point3f> point3ds(valid_cnt);
            vector<Point2f> points(valid_cnt);
            vector<Point2f> points_prev(valid_cnt);
            int j = 0;
            for (auto i = 0; i < status.size(); i++)
            {
                if (status[i])
                {
                    point3ds[j] = feat3ds[i];
                    points[j] = feats_next[i];
                    points_prev[j] = feats[i];
                    j++;
                }
            }
            Mat rvec, tvec;
            Mat dR;
            vector<uchar> inliers;
            bool ret = cv::solvePnPRansac(point3ds, points, camera_matrix, dist_coeffs,
                               rvec, tvec,
                               false, 30, 6.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
            cout << "rvec: " << rvec << endl;
            cout << "tvec: " << tvec << endl;
            //visualize_features(left_img, points_prev, points, inliers);
            if (ret)
            {
                cv::Rodrigues(rvec, dR);
            }
        }
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

void stereo_vo::update_keyframe(Mat &left_img, Mat &right_img)
{

#define USE_ORB_FEATURE
#ifdef USE_ORB_FEATURE
    Ptr<FeatureDetector> detector = cv::ORB::create();
    vector<KeyPoint> points;
    detector->detect(left_img, points);
    feats.resize(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        feats[i] =  points[i].pt;
    }
#else
    double quality_level = 0.01;
    int block_size = 3;
    bool use_harris = false;
    double k = 0.04;

    //feature detection
    cv::goodFeaturesToTrack(left_img,
                            feats,
                            max_feat_cnt,
                            quality_level,
                            min_feat_dist,
                            cv::Mat(),
                            block_size,
                            use_harris, k);
    cout << "good feature size: " << feats.size() << endl;

#endif


    stereo_triangulate(left_img, right_img, feats, feat3ds);
    left_img.copyTo(keyframe_img);

    qk = q;
    tk = t;
}

int stereo_vo::get_valid_feat_cnt(vector<uchar> &status)
{
    int cnt = 0;
    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            cnt++;
        }
    }
    return cnt;
}

void stereo_vo::visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar>& status)
{
    static Mat img_color;
    if (feat_vis_enable)
    {
        cv::cvtColor(img, img_color, cv::COLOR_GRAY2RGB);
        Scalar color = Scalar(0, 0, 255);
        Scalar color_prev = Scalar(255, 0, 0);
        Scalar color_next = Scalar(0, 255, 0);
        for (auto i = 0; i < feats.size(); i++)
        {
            if (status[i])
            {
                line(img_color, feats[i], feats_prev[i], color, 1, 8);
                circle(img_color, feats[i], 1, color);
                circle(img_color, feats_prev[i], 2, color_prev);
                circle(img_color, feats[i], 2, color_next);
            }
        }
        imshow("feats", img_color);
        waitKey(2);
    }
}

#endif

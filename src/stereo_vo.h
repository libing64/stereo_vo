#ifndef __STEREO_VO_H
#define __STEREO_VO_H
#include <iostream>
#include <Eigen/Eigen>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace Eigen;
using namespace cv;

static bool CvKeyPointResponseCompare(const cv::KeyPoint &p1,
                                      const cv::KeyPoint &p2)
{
    return (p1.response > p2.response);
}

class stereo_vo
{
private:
    int max_feat_cnt = 1000;
    int min_feat_cnt = 50;
    int min_track_cnt = 50;
    int min_feat_dist = 30;
    int min_disparity = 2;
    int max_epipolar = 5;
    bool feat_vis_enable = false;
    //camera matrix
    Matrix3d K;
    double baseline;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    Mat keyframe;
public:
    Quaterniond q;
    Vector3d t;
    double timestamp;

    //state of keyframe
    Quaterniond qk;
    Vector3d tk;

    int is_camera_info_init;
    vector<Point3f> feat3ds;
    vector<Point2f> feats;
    Mat masks;
    Mat feats_img;

    stereo_vo();
    ~stereo_vo();

    void set_camere_info(const sensor_msgs::CameraInfoConstPtr& msg);

    void remove_outliers(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f> &feat3ds);

    void stereo_detect(Mat &left_img, Mat &right_img);
    void stereo_visualize(Mat &left_img, Mat &right_img, vector<Point2f>&left_feats, vector<Point2f>&right_feats);

    int stereo_track(Mat& keyframe, Mat& img);

    void update(Mat& left_img, Mat& right_img);
    void visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar> &status);
};

stereo_vo::stereo_vo()
{
    is_camera_info_init = false;
    qk = Quaterniond::Identity();
    tk = Vector3d::Zero();
    q = qk;
    t = tk;
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

void stereo_vo::remove_outliers(vector<Point2f> &feats_prev, vector<Point2f> &feats_curr, vector<Point3f>& feat3ds)
{
    vector<uchar> status;
    Mat F = findFundamentalMat(feats_prev, feats_curr, FM_RANSAC, 1.0, 0.99, status);
    int j = 0;
    for (int i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            if (i != j)
            {
                feats_prev[j] = feats_prev[i];
                feats_curr[j] = feats_curr[i];
                feat3ds[j] = feat3ds[i];
            }
            j++;
        }
    }
    feats_prev.resize(j);
    feats_curr.resize(j);
    feat3ds.resize(j);
    cout << "inlier percent: " << (feats_prev.size() * 1.0) / status.size() << endl;
}

void stereo_vo::stereo_detect(Mat &left_img, Mat &right_img)
{
    vector<KeyPoint> left_keypoints;
    vector<Point2f> left_feats, right_feats;
    //1. orb feature
    //Ptr<FeatureDetector> detector = cv::ORB::create();
    //detector->detect(left_img, left_keypoints);
    //KeyPoint::convert(left_keypoints, left_feats);

    //2. fast feature
    int thresh = 10;
    Mat mask = cv::Mat(left_img.size(), CV_8UC1, cv::Scalar(255));
    Ptr<FastFeatureDetector> detector = FastFeatureDetector::create(thresh);
    detector->detect(left_img, left_keypoints);
    sort(left_keypoints.begin(), left_keypoints.end(), CvKeyPointResponseCompare);

    for (int i = 0; i < left_keypoints.size(); i++)
    {
        if (left_feats.size() < max_feat_cnt && mask.at<unsigned char>(left_keypoints[i].pt.y, left_keypoints[i].pt.x))
        {
            left_feats.push_back(left_keypoints[i].pt);
            circle(mask, left_keypoints[i].pt, min_feat_dist, cv::Scalar(0), cv::FILLED);
        }
    }

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

    vector<Point2f> feats_tracked(count);

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
            // cout << "dxdy: " << dx << " " << dy << endl;
            // cout << "p: " << p << endl;
            feat3ds[j] = p;
            feats[j] = left_feats[i];
            feats_tracked[j] = right_feats[i];
            j++;
        }
    }

    remove_outliers(feats, feats_tracked, feat3ds);

    //cout << "feat3ds: " << feat3ds << endl;
    left_img.copyTo(keyframe);
    qk = q;
    tk = t;
    cout << "stereo detect: " << qk.coeffs().transpose() << "  tk: " << t.transpose() << endl;
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

int stereo_vo::stereo_track(Mat &keyframe, Mat &img)
{
    vector<uchar> status;
    vector<float> err;
    vector<Point2f> feats_curr;
    cv::calcOpticalFlowPyrLK(keyframe, img, feats, feats_curr, status, err);
    int count = std::count(status.begin(), status.end(), 1);

    vector<Point3f> point3ds(count);
    vector<Point2f> points(count);

    vector<Point2f> points_curr(count);
    int j = 0;
    for (auto i = 0; i < status.size(); i++)
    {
        if (status[i])
        {
            point3ds[j] = feat3ds[i];
            points_curr[j] = feats_curr[i];
            points[j] = feats[i];
            j++;
        }
    }
    remove_outliers(points, points_curr, point3ds);

    Mat rvec, tvec;
    Mat dR;
    vector<uchar> inliers;
    bool ret = cv::solvePnPRansac(point3ds, points_curr, camera_matrix, dist_coeffs,
                                  rvec, tvec,
                                  false, 30, 6.0, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    cout << "rvec: " << rvec << endl;
    cout << "tvec: " << tvec << endl;
    visualize_features(img, points, points_curr, inliers);
    if (ret)
    {
        Mat rmat;
        cv::Rodrigues(rvec, rmat);

        Matrix3d dR;
        Quaterniond dq;
        Vector3d dt;

        cv2eigen(rmat, dR);
        cv2eigen(tvec, dt);

        dt = -dR.transpose() * dt;
        dq = Quaterniond(dR.transpose());
        t = tk + qk.toRotationMatrix() * dt;
        q = qk * dq;
        cout << "stereo track: " << q.coeffs().transpose() << "  t: " << t.transpose() << endl;
    }
    int inlier_count = std::count(inliers.begin(), inliers.end(), 1);
    return inlier_count;
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
        int inlier_count = stereo_track(keyframe, left_img);
        if (inlier_count < min_feat_cnt)
        {
            stereo_detect(left_img, right_img);
        }
    }
}

void stereo_vo::visualize_features(Mat &img, vector<Point2f> &feats, vector<Point2f> &feats_prev, vector<uchar>& status)
{
    static Mat img_color;
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
    img_color.copyTo(feats_img);
}

#endif

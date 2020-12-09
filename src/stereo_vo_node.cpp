#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "stereo_vo.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace Eigen;
using namespace cv;

ros::Subscriber camera_info_sub;
stereo_vo stereo;

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    if (!stereo.is_camera_info_init)
        stereo.set_camere_info(msg);
    camera_info_sub.shutdown();
}

void image_callback(const sensor_msgs::ImageConstPtr &left_image_msg,
                    const sensor_msgs::ImageConstPtr &right_image_msg)
{
    Mat left_img = cv_bridge::toCvCopy(left_image_msg, string("mono8"))->image;
    Mat right_img = cv_bridge::toCvCopy(left_image_msg, string("mono8"))->image;
    imshow("left", left_img);
    imshow("right", right_img);
    waitKey(2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_vo");
    ros::NodeHandle n("~");

    camera_info_sub = n.subscribe("/camera_info", 10, camera_info_callback);

    message_filters::Subscriber<sensor_msgs::Image> left_img_sub(n, "/left_image", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_img_sub(n, "/right_image", 2);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_img_sub, right_img_sub);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    ros::spin();
    return 0;
}
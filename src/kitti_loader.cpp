#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud2.h>

using std::cout;
using std::endl;

void readin_camera_info(std::string filename, Eigen::MatrixXd&P1, Eigen::MatrixXd& P2)
{
    P1 = Eigen::MatrixXd::Zero(3, 4);
    P2 = Eigen::MatrixXd::Zero(3, 4);
    cout << "filename: " << filename << endl;
    std::ifstream camera_info_file(filename, std::ifstream::in);
    std::string line;
    std::string s;


    std::getline(camera_info_file, line);
    std::stringstream camera_info_stream(line);

    std::getline(camera_info_stream, s, ' ');
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            std::getline(camera_info_stream, s, ' ');
            P1(i, j) = stof(s);
        }
    }
    std::getline(camera_info_file, line);
    camera_info_stream = std::stringstream(line);

    std::getline(camera_info_stream, s, ' ');
    for (std::size_t i = 0; i < 3; ++i)
    {
        for (std::size_t j = 0; j < 4; ++j)
        {
            std::getline(camera_info_stream, s, ' ');
            P2(i, j) = stof(s);
        }
    }
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, fixed_frame_id;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    n.getParam("fixed_frame_id", fixed_frame_id);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';

    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    ros::Publisher pub_left_camera_info = n.advertise<sensor_msgs::CameraInfo>("/left_camera_info", 2);
    ros::Publisher  pub_right_camera_info = n.advertise<sensor_msgs::CameraInfo>("/right_camera_info", 2);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry>("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = fixed_frame_id;
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path>("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = fixed_frame_id;

    std::string timestamp_path = "data_odometry_gray/dataset/sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "data_odometry_poses/dataset/poses/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    std::string camera_info_path = "data_odometry_calib/dataset/sequences/" + sequence_number + "/calib.txt";
    //std::ifstream camera_info_file(dataset_folder + ground_truth_path, std::ifstream::in);
    Eigen::MatrixXd P1, P2;
    readin_camera_info(dataset_folder + camera_info_path, P1, P2);
    sensor_msgs::CameraInfo left_camera_info, right_camera_info;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            left_camera_info.P[i * 4 + j] = P1(i, j);
            right_camera_info.P[i * 4 + j] = P2(i, j);
        }
    }
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            left_camera_info.K[i * 3 + j] = P1(i, j);
            right_camera_info.K[i * 3 + j] = P2(i, j);
        }
    }

    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "data_odometry_gray/dataset/sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);
        right_image_path << dataset_folder << "data_odometry_gray/dataset/sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), cv::IMREAD_GRAYSCALE);

        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);


        std_msgs::Header header;
        header.frame_id = "base_link";
        header.stamp = ros::Time(timestamp);

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);


        left_camera_info.header = header;
        right_camera_info.header = header;
        pub_left_camera_info.publish(left_camera_info);
        pub_right_camera_info.publish(right_camera_info);

        line_num++;
        r.sleep();
    }
    return 0;
}
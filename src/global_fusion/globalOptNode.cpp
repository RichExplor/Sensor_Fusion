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

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car;
nav_msgs::Path *global_path;
double last_loam_t = -1;
std::queue<nav_msgs::OdometryConstPtr> vioQueue;
std::mutex m_buf;


void Loam_Vio_callback(const nav_msgs::Odometry::ConstPtr& loam_msg, const nav_msgs::Odometry::ConstPtr& vio_msg)
{

    //printf("vio_callback! \n");
    double t = loam_msg->header.stamp.toSec();
    last_loam_t = t;

    double pose_x = loam_msg->pose.pose.position.x;  // 维度
    double pose_y = loam_msg->pose.pose.position.y;  // 经度
    double pose_z = loam_msg->pose.pose.position.z;  // 海拔
    // double pos_accuracy = _msg->position_covariance[0];  // 位置协方差
    double pos_accuracy = 1;  // 位置协方差 0.2
    if(pos_accuracy <= 0)
        pos_accuracy = 1;
    //printf("receive covariance %lf \n", pos_accuracy);
    globalEstimator.inputLOAM(t, pose_x, pose_y, pose_z, pos_accuracy);  // 将经纬度高度转换为xyz坐标

    // vio_msg processing
    double vio_t = vio_msg->header.stamp.toSec();
    printf("vio t: %f, loam t: %f \n", vio_t, t);
    // 10ms sync tolerance  0.05
    if(vio_t >= t - 0.01 && vio_t <= t + 0.01)
    {
        Eigen::Vector3d vio_t(vio_msg->pose.pose.position.x, vio_msg->pose.pose.position.y, vio_msg->pose.pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = vio_msg->pose.pose.orientation.w;
        vio_q.x() = vio_msg->pose.pose.orientation.x;
        vio_q.y() = vio_msg->pose.pose.orientation.y;
        vio_q.z() = vio_msg->pose.pose.orientation.z;
        globalEstimator.inputOdom(t, vio_t, vio_q);   // 获取后端的VIO位姿，送入到全局位姿中。 变换到当前坐标下
    }
    

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;
    odometry.header = loam_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);

    // write result to file
    std::ofstream foutC("/home/lenovo/output/global_pose.txt", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(9);
    foutC << loam_msg->header.stamp.toSec() << " ";
    foutC.precision(5);
    foutC << global_t.x() << " "
        << global_t.y() << " "
        << global_t.z() << " "
        << global_q.x() << " "
        << global_q.y() << " "
        << global_q.z() << " "
        << global_q.w() << endl;
    foutC.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;

    message_filters::Subscriber<nav_msgs::Odometry> sub_loam(n, "/Odometry", 100);  // /Odometry
    message_filters::Subscriber<nav_msgs::Odometry> sub_vio(n, "/Sensor_fusion_visual_eatimator/odometry", 100);
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), sub_loam, sub_vio);
    sync.registerCallback(boost::bind(&Loam_Vio_callback, _1, _2));


    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}

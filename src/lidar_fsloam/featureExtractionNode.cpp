#include <iostream>
#include <mutex>
#include <thread>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud.h>  // 点云消息
#include <sensor_msgs/NavSatFix.h>   // GPS消息
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "include/featureExtraction.hpp"

using namespace std;
using namespace cv;


featureExtraction featureExtractFactor;

std::mutex mutex_velodyne; 
std::queue<sensor_msgs::PointCloud2> lidarQueue;

ros::Publisher pubFilterCloud, pubEdgeCloud, pubSurfCloud;

pcl::VoxelGrid<PointType> voxelFilter;

double extractMeanTime = 0;
int lidarCount = 0;

void callbackVelodyne(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    mutex_velodyne.lock();
    lidarQueue.push(*cloudMsg);
    mutex_velodyne.unlock();

}

void lidar_process()
{
    while(1)
    {
       if(!lidarQueue.empty())
       {
           // 读取原始点云信息与时间戳
           mutex_velodyne.lock();
           pcl::PointCloud<PointType>::Ptr pointCloud_In(new pcl::PointCloud<PointType>());
           pcl::fromROSMsg(lidarQueue.front(), *pointCloud_In);
           ros::Time pointCloud_Time = (lidarQueue.front()).header.stamp;
           sensor_msgs::PointCloud2 lidarCloudMsg;
           lidarCloudMsg = std::move(lidarQueue.front());
           lidarQueue.pop();
           mutex_velodyne.unlock();

           // 点云特征提取--边缘点/平面点
           TicToc start_feature;
           pcl::PointCloud<PointType>::Ptr featureCloud_Edge(new pcl::PointCloud<PointType>());
           pcl::PointCloud<PointType>::Ptr featureCloud_Surf(new pcl::PointCloud<PointType>());

           lidarCount++;
           featureExtractFactor.extractFeature(pointCloud_In, featureCloud_Edge, featureCloud_Surf);
           extractMeanTime += start_feature.toc();
           std::cout<<"point cloud feature extract time is " << extractMeanTime / lidarCount << " ms" <<endl;

           // 发布点云--滤波后点云/边缘点/平面点
           pcl::PointCloud<PointType>::Ptr pointCloud_filter(new pcl::PointCloud<PointType>());

           voxelFilter.setInputCloud(pointCloud_In);
           voxelFilter.filter(*pointCloud_filter);

           sensor_msgs::PointCloud2 filterCloudMsg;
           pcl::toROSMsg(*pointCloud_In, filterCloudMsg);
           filterCloudMsg.header.stamp = pointCloud_Time;
           filterCloudMsg.header.frame_id = "base_link";
           pubFilterCloud.publish(filterCloudMsg);

           sensor_msgs::PointCloud2 edgeCloudMsg;
           pcl::toROSMsg(*featureCloud_Edge, edgeCloudMsg);
           edgeCloudMsg.header.stamp = pointCloud_Time;
           edgeCloudMsg.header.frame_id = "base_link";
           pubEdgeCloud.publish(edgeCloudMsg);

           sensor_msgs::PointCloud2 surfCloudMsg;
           pcl::toROSMsg(*featureCloud_Surf, surfCloudMsg);
           surfCloudMsg.header.stamp = pointCloud_Time;
           surfCloudMsg.header.frame_id = "base_link";
           pubSurfCloud.publish(surfCloudMsg);

       } 

       //sleep 2ms
       std::chrono::milliseconds dura(2);
       std::this_thread::sleep_for(dura);

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MultiSensor_fusion_featureExtractionNode");
    ros::NodeHandle nh;

    // load patamerater
    std::string pointCloutToplic;
    double voxelSize;
    double edgeMapLeafSize;
    double surfMapLeafSize;

    nh.param<std::string>("lidarTolic", pointCloutToplic, "/velodyne_points");
    nh.param<double>("voxelSize", voxelSize, 0.2);
    
    voxelFilter.setLeafSize(voxelSize, voxelSize, voxelSize);

    // initizer feature extractor
    featureExtractFactor.initParam(nh);

    ros::Subscriber subLidarCloud =  nh.subscribe<sensor_msgs::PointCloud2>(pointCloutToplic, 100, callbackVelodyne);

    pubFilterCloud = nh.advertise<sensor_msgs::PointCloud2>("/pointclude_filter", 100);
    pubEdgeCloud = nh.advertise<sensor_msgs::PointCloud2>("/pointCloud_Edge", 100);
    pubSurfCloud = nh.advertise<sensor_msgs::PointCloud2>("/pointCloud_Surf", 100);

    std::thread velodyne_thread {lidar_process};

    ros::spin();

    return 0;
}
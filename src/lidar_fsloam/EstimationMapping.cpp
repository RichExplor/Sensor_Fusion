#include "include/EstimationMapping.hpp"

#include <thread>
#include <mutex>
#include <queue>
#include <fstream>

using namespace std;

std::string poseSaveTum;
double firstTimes = 0.0;

EstimationMapping Estimator;
mutex est_mutex;
bool systemInited = false;

queue<sensor_msgs::PointCloud2ConstPtr> fullQueue;
queue<sensor_msgs::PointCloud2ConstPtr> edgeQueue;
queue<sensor_msgs::PointCloud2ConstPtr> surfQueue;

ros::Publisher pubOdometry, pubGlobalPath, pubGlobalMap, pubFullCloud;
nav_msgs::Path laserPath;

double optiMeanTime = 0;
int lidarCount = 0;

void callbackFull(const sensor_msgs::PointCloud2ConstPtr& fullMsg)
{
    est_mutex.lock();
    fullQueue.push(fullMsg);
    est_mutex.unlock();
}

void callbackEdge(const sensor_msgs::PointCloud2ConstPtr& edgeMsg)
{
    est_mutex.lock();
    edgeQueue.push(edgeMsg);
    est_mutex.unlock();
}

void callbackSurf(const sensor_msgs::PointCloud2ConstPtr& surfMsg)
{
    est_mutex.lock();
    surfQueue.push(surfMsg);
    est_mutex.unlock();
}


void ProcessEstimator()
{
    while(1)
    {
        if(!surfQueue.empty() && !edgeQueue.empty() && !fullQueue.empty())
        {
            
            // 同步特征点云数据
            est_mutex.lock();
            if( !surfQueue.empty() && ( surfQueue.front()->header.stamp.toSec() < edgeQueue.front()->header.stamp.toSec() ) )
            {
                surfQueue.pop();
                cout<<"optimation time  surfQueue"<<endl;
                est_mutex.unlock();

                continue;
            }

            if( !edgeQueue.empty() && ( edgeQueue.front()->header.stamp.toSec() < surfQueue.front()->header.stamp.toSec() ) )
            {
                edgeQueue.pop();
                cout<<"optimation time  edgeQueue"<<endl;
                est_mutex.unlock();

                continue;
            }

            // 经过上述数据同步，读取数据
            pcl::PointCloud<PointType>::Ptr MapCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr fullCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr edgePointCloudIn(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr surfPointCloudIn(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullQueue.front(), *fullCloud);
            pcl::fromROSMsg(*surfQueue.front(), *surfPointCloudIn);
            pcl::fromROSMsg(*edgeQueue.front(), *edgePointCloudIn);
            ros::Time currentPointTime = surfQueue.front()->header.stamp;
            fullQueue.pop();
            surfQueue.pop();
            edgeQueue.pop();
            est_mutex.unlock();

            if(firstTimes <= 0.01)
                firstTimes = currentPointTime.toSec();

            if(systemInited == false)
            {
                Estimator.localMapInited(edgePointCloudIn, surfPointCloudIn);
                systemInited = true;
                cout<<"system is inited."<<endl;
            }
            else{

                lidarCount++;

                TicToc optimation_time;
                // 点/面特征约束优化
                Estimator.optimation_processing(edgePointCloudIn, surfPointCloudIn);

                Estimator.getMapCloud(MapCloud);

                optiMeanTime += optimation_time.toc();
                cout<<"optimation time "<<optiMeanTime / lidarCount<<" ms"<<endl;
            }

            // publish current pose
            Eigen::Quaterniond q_estimator(Estimator.globalOdom.rotation());
            Eigen::Vector3d t_estimator = Estimator.globalOdom.translation();

            // publish tf
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(t_estimator.x(), t_estimator.y(), t_estimator.z()) );
            tf::Quaternion q_tf(q_estimator.x(), q_estimator.y(), q_estimator.z(), q_estimator.w());
            transform.setRotation(q_tf);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link"));

            // publish Odometry and Path
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "world";
            laserOdometry.child_frame_id  = "base_link";
            laserOdometry.header.stamp = currentPointTime;
            laserOdometry.pose.pose.orientation.x = q_estimator.x();
            laserOdometry.pose.pose.orientation.y = q_estimator.y();
            laserOdometry.pose.pose.orientation.z = q_estimator.z();
            laserOdometry.pose.pose.orientation.w = q_estimator.w();
            laserOdometry.pose.pose.position.x    = t_estimator.x();
            laserOdometry.pose.pose.position.y    = t_estimator.y();
            laserOdometry.pose.pose.position.z    = t_estimator.z();
            pubOdometry.publish(laserOdometry);

            // save pose kitti 
			std::ofstream foutc(poseSaveTum + "result_fs_loam.txt", std::ios::app);
			foutc.setf(std::ios::fixed, std::ios::floatfield);
			foutc.precision(9);
			// foutc << (currentPointTime.toSec() - firstTimes)<< " ";
            foutc << currentPointTime.toSec()<< " ";
			foutc.precision(5);
			foutc << t_estimator.x()<<" "
				  << t_estimator.y()<<" "
				  << t_estimator.z()<<" "
				  << q_estimator.x()<<" "
				  << q_estimator.y()<<" "
				  << q_estimator.z()<<" "
				  << q_estimator.w()<<std::endl;
			foutc.close();

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose   = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.header.frame_id = "world";
            laserPath.poses.push_back(laserPose);
            pubGlobalPath.publish(laserPath);

            // publish local map
            sensor_msgs::PointCloud2 MapCloudMsg;
            pcl::toROSMsg(*MapCloud, MapCloudMsg);
            MapCloudMsg.header.stamp = currentPointTime;
            MapCloudMsg.header.frame_id = "world";
            pubGlobalMap.publish(MapCloudMsg);

            // publish no local map
            sensor_msgs::PointCloud2 fullCloudMsg;
            pcl::toROSMsg(*fullCloud, fullCloudMsg);
            fullCloudMsg.header.stamp = currentPointTime;
            fullCloudMsg.header.frame_id = "world";
            pubFullCloud.publish(fullCloudMsg);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "MultiSensor_fusion_EstimationMapping");
    ros::NodeHandle nh;

    Estimator.initParameter(nh);

    nh.param<string>("poseSaveTum", poseSaveTum, "/home/lenovo/output/TaoZi/");

    ros::Subscriber subFullCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pointclude_filter", 100, callbackFull);
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pointCloud_Edge", 100, callbackEdge);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pointCloud_Surf", 100, callbackSurf);

    pubOdometry = nh.advertise<nav_msgs::Odometry>("/Odometry", 100);
    pubGlobalPath = nh.advertise<nav_msgs::Path>("/path", 100);
    pubGlobalMap  = nh.advertise<sensor_msgs::PointCloud2>("/GlobalMap", 100);
    pubFullCloud = nh.advertise<sensor_msgs::PointCloud2>("/fullCloud", 100);

    std::thread EstimateProcess{ProcessEstimator};

    ros::spin();

    return 0;
}
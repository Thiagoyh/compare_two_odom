
#pragma once

#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "options.h"

//points covariance class
class Double2d{
public:
	int id;
	double value;
	Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};


class scanRegistration
{
    public:
    	scanRegistration() = default;
		scanRegistration(common::LidarOptions &lidar_param);

		void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		void laser_processing();
		void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in,
							   pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_edge,
							   pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out_surf);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
		                                 std::vector<Double2d>& cloudCurvature,
		                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
		                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

        ros::NodeHandle nh_;
		ros::Subscriber subLaserCloud;
		ros::Publisher pubEdgePoints;
        ros::Publisher pubSurfPoints;
        ros::Publisher pubLaserCloudFiltered;
	private:
     	common::LidarOptions lidar_param_;
		std::mutex mutex_lock;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

		double total_time =0;
        int total_frame=0;
};





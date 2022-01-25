
#pragma once

//std lib
#include <string>
#include <math.h>
#include <vector>
#include <mutex>
#include <string>
#include <fstream>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "pose_graph.h"
#include "graph_node.h"
#include "options.h"
#include "submap/activeSubmap.h"

#include "scan_matcher/scan_matcher.h"
#include "scan_matcher/ceres_cost_function.h"
#include "floam/save_pcd.h"

class LocalTrajectoryBuilder
{

    public:
	    LocalTrajectoryBuilder();
    	LocalTrajectoryBuilder(const common::OdomOptions& node_options);

        // 调用主函数
		void odom_estimation();
		// 回调函数
		void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
		bool savePcdService(floam::save_pcdRequest& req, floam::save_pcdRequest& res);
		std::shared_ptr<const Submap> getMap();
		// ros节点与消息发布
        ros::NodeHandle nh;
        ros::Publisher pubLaserOdometry;
        ros::Publisher pubSubmap_corner;
        ros::Publisher pubSubmap_surf;
        ros::Publisher pubMap;
		ros::ServiceServer srvSavePcd;

	private:

        // 前端匹配位姿
        Eigen::Isometry3d odom;
		Eigen::Isometry3d last_odom;
		// local map
		// 前端维护两个子图， 保存的是相邻两个子图的shared_ptr共享指针
		ActiveSubmaps active_submaps_;
		// ceres匹配器
		optimization::CeresScanMatcher ceres_scan_matcher;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;
		pcl::VoxelGrid<pcl::PointXYZI> downSizeFullMap;

        // 后端
		common::ThreadPool thread_pool_;
		std::unique_ptr<mapping::PoseGraph> pose_graph_;

		void addPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud,
		                    const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud);

		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in,
		                       pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out,
		                       const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in,
		                       pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out);
		void updatePointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
		                      const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in);
		void publishRosTf(const ros::Time& pointcloud_time);
		void publishPointCloud(
			const pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_publish,
			const ros::Time& pointcloud_time,
			const ros::Publisher& pub);

		// 发布与订阅的消息
		std::mutex mutex_lock;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
        std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
		pcl::PointCloud<pcl::PointXYZI>::Ptr fullMap;
		ros::Subscriber subEdgeLaserCloud;
		ros::Subscriber subSurfLaserCloud;

		// odom_estimation的参数
		bool is_odom_inited = false;
        double total_time = 0;
        int total_frame = 0;
		double scan_period = 0.1;
		//optimization count
		int optimization_count;

		//运动过滤参数
		double speed_filter = 0.1;
		// tag: 子图个数
		int num_of_submaps = 0;
		int num_of_pointcloud = 5;
		std::string savePcdDirectory = "/home/xcy/pcd/pointcloud";
		std::ofstream fout;
		std::ofstream fout2;
};

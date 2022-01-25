#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


#include "transform.h"

class Submap
{
public:
    Submap(const transform::Rigid3d& local_submap_pose)
    : local_pose_(local_submap_pose),
     laserCloudCornerMap_(new pcl::PointCloud<pcl::PointXYZI>()),
     laserCloudSurfMap_(new pcl::PointCloud<pcl::PointXYZI>())
     {
         downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
         downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);
     }

    //返回local坐标系子图坐标
    transform::Rigid3d local_pose() const { return local_pose_; }

    // 插入点云子图中的雷达数据数
    int num_range_data() const { return num_range_data_; }
    void set_num_range_data(const int num_range_data)
    {
        num_range_data_ = num_range_data;
    }

    // 返回完成状态
    bool insertion_finished() const { return insertion_finished_;  }

    void set_insertion_finished(bool insertion_finished)
    {
        insertion_finished_ = insertion_finished;
    }

    // 将雷达数据写入到地图，传入原点位于匹配位姿处的点云
    void InsertRangeData(const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud,
                         const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr conerMap() const;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfMap() const;
    // 结束子图

    void Finish() { set_insertion_finished(true); }

private:
    double map_resolution = 0.4;
    const transform::Rigid3d local_pose_;
    int num_range_data_ = 0;
    bool insertion_finished_ = false;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCornerMap_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurfMap_;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterEdge;
	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterSurf;

};

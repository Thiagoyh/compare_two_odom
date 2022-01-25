#include "submap/submap.h"

pcl::PointCloud<pcl::PointXYZI>::Ptr Submap::conerMap() const
{
    return laserCloudCornerMap_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr Submap::surfMap() const
{
    return laserCloudSurfMap_;
}


void Submap::InsertRangeData(const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledEdgeCloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& downsampledSurfCloud)
    {
        *laserCloudCornerMap_ += *downsampledEdgeCloud;
        *laserCloudSurfMap_ += *downsampledSurfCloud;

        downSizeFilterSurf.setInputCloud(laserCloudSurfMap_);
        downSizeFilterSurf.filter(*laserCloudSurfMap_);
        downSizeFilterEdge.setInputCloud(laserCloudSurfMap_);
        downSizeFilterEdge.filter(*laserCloudCornerMap_);

        set_num_range_data(num_range_data() + 1);

    }
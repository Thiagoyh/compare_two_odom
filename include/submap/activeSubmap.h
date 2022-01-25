#pragma once

#include "submap/submap.h"
#include "glog/logging.h"

class ActiveSubmaps
{
public:
  ActiveSubmaps();
  ActiveSubmaps(const ActiveSubmaps&) = delete;
  ActiveSubmaps& operator=(const ActiveSubmaps&) = delete;

  std::vector<std::shared_ptr<const Submap>> InsertRangeData(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr & downsampledEdgeCloud,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud,
      transform::Rigid3d pose_esitimate);

  std::vector<std::shared_ptr<const Submap>> submaps() const;

  void set_num_range_data(int num_range_data_);

private:
  int num_range_data = 90;
  void FinishSubmap();
  void AddSubmap(const transform::Rigid3d &origin);

  std::vector<std::shared_ptr<Submap>> submaps_;

};
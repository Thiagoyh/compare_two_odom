#include "submap/activeSubmap.h"

ActiveSubmaps::ActiveSubmaps(){
}

std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::submaps() const
{
   return std::vector<std::shared_ptr<const Submap>>(submaps_.begin(),
                                                      submaps_.end());
}

std::vector<std::shared_ptr<const Submap>> ActiveSubmaps::InsertRangeData(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledEdgeCloud,
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &downsampledSurfCloud,
      transform::Rigid3d pose_esitimate)
      {
      if (submaps_.empty() ||
          submaps_.back()->num_range_data() == num_range_data) {
          AddSubmap(pose_esitimate);
       }
       for(auto& submap : submaps_){
           submap->InsertRangeData(downsampledEdgeCloud,
                                  downsampledSurfCloud);
       }

       if(submaps_.front()->num_range_data() == 2 * num_range_data)
       {

           submaps_.front()->Finish();
       }
       return submaps();
      }

void ActiveSubmaps::AddSubmap(const transform::Rigid3d &origin)
{
    if(submaps_.size() >= 2)
    {
        CHECK(submaps_.front()->insertion_finished());
        submaps_.erase(submaps_.begin());
    }

    submaps_.push_back(std::make_shared<Submap>(origin));
}

void ActiveSubmaps::set_num_range_data(int num_range_data_){
    num_range_data = num_range_data_;
}
#pragma once

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <vector>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include "common/task.h"
#include "common/thread_pool.h"
#include "graph_node.h"
#include "submap/submap.h"
#include "histogram.h"
#include "scan_matcher/scan_matcher.h"
#include "pose_graph_interface.h"
#include "scan_matcher/ceres_pose.h"

namespace optimization{
class ConstraintBuilder {
 public:
  using Constraint = mapping::PoseGraphInterface::Constraint;
  using Result = std::vector<mapping::PoseGraphInterface::Constraint>;

  ConstraintBuilder(common::ThreadPoolInterface* thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id', and the 'compressed_point_cloud' for 'node_id'. The
  // 'initial_relative_pose' is relative to the 'submap'.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddConstraint(const mapping::SubmapId& submap_id, const Submap* submap,
                          const mapping::NodeId& node_id,
                          const mapping::TrajectoryNode::Data* const constant_data,
                          const transform::Rigid3d& global_node_pose,
                          const transform::Rigid3d& global_submap_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_id' and the 'compressed_point_cloud' for 'node_id'.
  // This performs full-submap matching.
  //
  // The pointees of 'submap' and 'compressed_point_cloud' must stay valid until
  // all computations are finished.
  void MaybeAddGlobalConstraint(
      const mapping::SubmapId submap_id, const Submap* submap, const mapping::NodeId node_id,
      const mapping::TrajectoryNode::Data* const constant_data);

  // Must be called after all computations related to one node have been added.
  void NotifyEndOfNode();

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by 'MaybeAdd*Constraint' have finished.
  // 'callback' is executed in the 'ThreadPool'.
  void WhenDone(const std::function<void(const Result&)>& callback);

  // Returns the number of consecutive finished nodes.
  int GetNumFinishedNodes();

  // Delete data related to 'submap_id'.
  void DeleteScanMatcher(const mapping::SubmapId submap_id);

  //static void RegisterMetrics(metrics::FamilyFactory* family_factory);



  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'compressed_point_cloud' do not change
  // anymore. As output, it may create a new Constraint in 'constraint'.
  void ComputeConstraint(const mapping::SubmapId& submap_id, const mapping::NodeId& node_id,
                         bool match_full_map,
                         const mapping::TrajectoryNode::Data* const constant_data,
                         const transform::Rigid3d& global_node_pose,
                         const transform::Rigid3d& global_submap_pose,
                         const Submap* submap,
                         std::unique_ptr<Constraint>* constraint)
      LOCKS_EXCLUDED(mutex_);

private:
  void RunWhenDoneCallback() LOCKS_EXCLUDED(mutex_);

  //const constraints::proto::ConstraintBuilderOptions options_;
  common::ThreadPoolInterface* thread_pool_;
  absl::Mutex mutex_;

  // 'callback' set by WhenDone().
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // TODO(gaschler): Use atomics instead of mutex to access these counters.
  // Number of the node in reaction to which computations are currently
  // added. This is always the number of nodes seen so far, even when older
  // nodes are matched against a new submap.
  int num_started_nodes_ GUARDED_BY(mutex_) = 0;

  int num_finished_nodes_ GUARDED_BY(mutex_) = 0;

  std::unique_ptr<common::Task> finish_node_task_ GUARDED_BY(mutex_);

  std::unique_ptr<common::Task> when_done_task_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries. Constraint search results
  // with below-threshold scores are also 'nullptr'.
  // 正在计算的约束的队列
  std::deque<std::unique_ptr<mapping::PoseGraphInterface::Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of dispatched or constructed scan matchers by 'submap_id'.

  CeresScanMatcher ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
  double max_constraint_distance = 30.0;

  // options
  bool log_matches = false;

  // ceres优化次数
  int optimization_count = 2;
};
}
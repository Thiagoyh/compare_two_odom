#pragma once

#include "graph_node.h"
#include "optimization_problem.h"
#include "submap/submap.h"

namespace mapping{

  enum class SubmapState { kNoConstraintSearch, kFinished };

  struct InternalTrajectoryState {
  enum class DeletionState {
    NORMAL,
    SCHEDULED_FOR_DELETION,
    WAIT_FOR_DELETION
  };

  PoseGraphInterface::TrajectoryState state =
      PoseGraphInterface::TrajectoryState::ACTIVE;
  DeletionState deletion_state = DeletionState::NORMAL;
};

  struct InternalSubmapData {
  std::shared_ptr<const Submap> submap;
  SubmapState state = SubmapState::kNoConstraintSearch;

  // IDs of the nodes that were inserted into this map together with
  // constraints for them. They are not to be matched again when this submap
  // becomes 'kFinished'.
  // 插入到此地图中的节点的 ID
  // 当此子图变为“kFinished”后, 这些节点将不再与这个地图进行匹配.
  std::set<NodeId> node_ids;
};
  struct PoseGraphData{
          MapById<SubmapId/*submapid*/, InternalSubmapData> submap_data;

          MapById<SubmapId /*submapid*/, optimization::SubmapSpec> global_submap_poses;
          MapById<NodeId /*nodeid*/, TrajectoryNode> trajectory_nodes;
          // 轨迹的状态
          InternalTrajectoryState trajectories_state;
          //节点的个数
          int num_trajectory_nodes = 0;
          std::vector<PoseGraphInterface::Constraint> constraints;
  };
}
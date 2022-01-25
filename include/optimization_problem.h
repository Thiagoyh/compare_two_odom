#pragma once

#include "pose_graph_interface.h"
#include "transform.h"
#include "graph_node.h"
namespace optimization
{

     //constexpr int trajectory_id = 0;
     struct SubmapSpec {
          transform::Rigid3d global_pose;
     };

     struct NodeSpec{
          transform::Rigid3d local_pose_3d;
          transform::Rigid3d global_pose_3d;
     };
     class OptimizationProblem{
     public:
         explicit OptimizationProblem() = default;
         ~OptimizationProblem();

         OptimizationProblem(const OptimizationProblem &) = delete;
         OptimizationProblem &operator=(const OptimizationProblem &) = delete;

         void AddTrajectoryNode(int trajectory_id,
                               const NodeSpec &node_data);
         void InsertTrajectoryNode(const mapping::NodeId& node_id, /*in node_id*/
                                   const NodeSpec &node_data);
         void TrimTrajectoryNode(const mapping::NodeId node_id);

         void AddSubmap(int trajectory_id,
                        const transform::Rigid3d &global_submap_pose);
         void InsertSubmap(const mapping::SubmapId& submap_id,
                           const transform::Rigid3d &global_submap_pose);
         void TrimSubmap(const mapping::SubmapId& submap_id);
         void SetMaxNumIterations(int32_t max_num_iterations);

         void Solve(
             const std::vector<mapping::PoseGraphInterface::Constraint> &constraints,
          const mapping::PoseGraphInterface::TrajectoryState& trajectory_state);

         const mapping::MapById<mapping::NodeId/*node_id*/, NodeSpec>& node_data() const {
         return node_data_;
         }
         const mapping::MapById<mapping::SubmapId/*submap_id*/, SubmapSpec>& submap_data() const {
         return submap_data_;
         }
  private:
         mapping::MapById<mapping::NodeId, NodeSpec> node_data_;                   // 节点坐标列表
         mapping::MapById<mapping::SubmapId, SubmapSpec> submap_data_;             // submap原点坐标列表

         //mapping::PoseGraphInterface::TrajectoryData trajectory_data_;
       };


}//namespace optimization
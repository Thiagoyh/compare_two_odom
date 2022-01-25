#include "optimization_problem.h"

namespace optimization{

    OptimizationProblem::~OptimizationProblem(){}

    void OptimizationProblem::AddSubmap(int trajectory_id,
                           const transform::Rigid3d &global_submap_pose){
        submap_data_.Append(trajectory_id, SubmapSpec{global_submap_pose});
    }

    void OptimizationProblem::InsertSubmap(const mapping::SubmapId& submap_id,
                           const transform::Rigid3d &global_submap_pose){
        submap_data_.Insert(submap_id, SubmapSpec{global_submap_pose});
    }

    void OptimizationProblem::AddTrajectoryNode(int trajectory_id,
                                  const NodeSpec& node_data){
        node_data_.Append(trajectory_id, node_data);
    }

    void OptimizationProblem::Solve(
             const std::vector<mapping::PoseGraphInterface::Constraint> &constraints,
          const mapping::PoseGraphInterface::TrajectoryState& trajectory_state){

          }
}//
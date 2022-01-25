#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "absl/synchronization/mutex.h"
#include "optimization_problem.h"
#include "common/thread_pool.h"
#include "graph_node.h"
#include "common/thread_pool.h"
#include "constraint_builder.h"
#include "pose_grapher_data.h"
#include "pose_graph_interface.h"
#include "glog/logging.h"
#include "common/time.h"

namespace mapping{

    constexpr int trajectory_id_ = 0;
    class PoseGraph : public PoseGraphInterface
    {
    public:
        PoseGraph();
        PoseGraph(std::unique_ptr<optimization::OptimizationProblem> optimization_problem,
                  common::ThreadPool *thread_pool);

        ~PoseGraph();
        PoseGraph(const PoseGraph &) = delete;
        PoseGraph &operator=(const PoseGraph &) = delete;

        NodeId AddNode(
            std::shared_ptr<const TrajectoryNode::Data> constant_data,
            const std::vector<std::shared_ptr<const Submap>> &insertion_submaps) LOCKS_EXCLUDED(mutex_);

        std::vector<Constraint> constraints() const LOCKS_EXCLUDED(mutex_);
        // std::map<int, TrajectoryData> GetTrajectoryData() const;

        transform::Rigid3d GetLocalToGlobalTransform(const int trajectory_id) const
            LOCKS_EXCLUDED(mutex_);

    private:
        void AddWorkItem(const std::function<WorkItem::Result()> &work_item)
            LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

        NodeId AppendNode(
            std::shared_ptr<const TrajectoryNode::Data> constant_data,
            int trajecory_id,
            const std::vector<std::shared_ptr<const Submap>> &insertion_submaps,
            const transform::Rigid3d &optimized_pose) LOCKS_EXCLUDED(mutex_);

        std::vector<SubmapId> InitializeGlobalSubmapPoses(
            int trajectory_id,
            const std::vector<std::shared_ptr<const Submap>>& insertion_submaps)
            EXCLUSIVE_LOCKS_REQUIRED(mutex_);

        WorkItem::Result ComputeConstraintsForNode(
        const NodeId& node_id,
        std::vector<std::shared_ptr<const Submap>> insertion_submaps,
        bool newly_finished_submap) LOCKS_EXCLUDED(mutex_);

        void ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id)
        LOCKS_EXCLUDED(mutex_);

        // TODO :
        void HandleWorkQueue(const std::vector<Constraint>& result)
        LOCKS_EXCLUDED(mutex_) LOCKS_EXCLUDED(work_queue_mutex_);

        void DrainWorkQueue() LOCKS_EXCLUDED(mutex_)
        LOCKS_EXCLUDED(work_queue_mutex_);

        // TODO :
        void WaitForAllComputations() LOCKS_EXCLUDED(mutex_)
        LOCKS_EXCLUDED(work_queue_mutex_);

        // TODO :
        void RunOptimization() LOCKS_EXCLUDED(mutex_);

        // TODO :
        //bool CanAddWorkItemModifying()
        //EXCLUSIVE_LOCKS_REQUIRED(mutex_);

        transform::Rigid3d ComputeLocalToGlobalTransform(
        const MapById<SubmapId, optimization::SubmapSpec>& global_submap_poses,
        const int trajectory_id)
        const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

        struct SubmapData {
           std::shared_ptr<const Submap> submap;
           transform::Rigid3d pose;
        };

        // TODO:
        MapById<SubmapId, SubmapData> GetSubmapDataUnderLock(const SubmapId submap_id) const
        EXCLUSIVE_LOCKS_REQUIRED(mutex_);

        using GlobalSlamOptimizationCallback =
        std::function<void(const int /*node_id*/,
                         const int /* submap_id */)>;

        GlobalSlamOptimizationCallback global_slam_optimization_callback_;

        // 只有这两个线程互斥锁
        mutable absl::Mutex mutex_;
        absl::Mutex work_queue_mutex_;

        std::unique_ptr<WorkQueue> work_queue_ GUARDED_BY(work_queue_mutex_);


        // Number of nodes added since last loop closure.
        int num_nodes_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;


        std::unique_ptr<optimization::OptimizationProblem> optimization_problem_;
        optimization::ConstraintBuilder constraint_builder_;

        common::ThreadPool* const thread_pool_;

        PoseGraphData data_ GUARDED_BY(mutex_);

        // 旋转、平移的权重
        double matcher_translation_weight = 500.0;
        double matcher_rotation_weight = 1600.0;
        int optimize_every_n_nodes = 120;
    };

}//namespace mapping
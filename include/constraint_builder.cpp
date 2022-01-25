#include "constraint_builder.h"

namespace optimization{

     ConstraintBuilder::ConstraintBuilder(
         common::ThreadPoolInterface* thread_pool):
         thread_pool_(thread_pool),
         finish_node_task_(absl::make_unique<common::Task>()),
         when_done_task_(absl::make_unique<common::Task>()){

         }

    ConstraintBuilder::~ConstraintBuilder() {
        absl::MutexLock locker(&mutex_);
        CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
        CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
        CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
        CHECK_EQ(num_started_nodes_, num_finished_nodes_);
        CHECK(when_done_ == nullptr);
    }

    void ConstraintBuilder::MaybeAddConstraint(
        const mapping::SubmapId& submap_id, const Submap* const submap,
        const mapping::NodeId& node_id, const mapping::TrajectoryNode::Data* const constant_data,
        const transform::Rigid3d& global_node_pose,
        const transform::Rigid3d& global_submap_pose
    ){
        if((global_node_pose.translation() - global_submap_pose.translation()).norm()
        > max_constraint_distance)
        {
             return;
        }

        absl::MutexLock locker(&mutex_);
        if(when_done_){
             LOG(WARNING)
                 << "MaybeAddConstraint was called while WhenDone was scheduled.";
        }
        constraints_.emplace_back();
        auto* const constraint = &constraints_.back();
        auto constraint_task = absl::make_unique<common::Task>();
        constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
        ComputeConstraint(submap_id, node_id, false, /* match_full_submap */
                      constant_data, global_node_pose, global_submap_pose,
                      submap, constraint);
        });
        auto constraint_task_handle =
        thread_pool_->Schedule(std::move(constraint_task));
        finish_node_task_->AddDependency(constraint_task_handle);
    }
   // todo:
   void ConstraintBuilder::ComputeConstraint(const mapping::SubmapId& submap_id,
                         const mapping::NodeId& node_id,
                         bool match_full_map,
                         const mapping::TrajectoryNode::Data* const constant_data,
                         const transform::Rigid3d& global_node_pose,
                         const transform::Rigid3d& global_submap_pose,
                         const Submap* submap,
                         std::unique_ptr<Constraint>* constraint){
         if(match_full_map)
         {
             LOG(WARNING) << "match_full_map: i'm so soory to tell i can't do it";
             return;
         }
         // constant_data当前帧，submap是地图,也就是做constant_data和submap的匹配
         if ((submap->local_pose().translation() -
                constant_data->local_pose.translation()).norm() > 70.0)
             return;
         pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(
             new pcl::PointCloud<pcl::PointXYZI>());
         pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
             new pcl::PointCloud<pcl::PointXYZI>());
         *target_cloud = *(submap->conerMap()) + *(submap->surfMap());
         *input_cloud = *(constant_data->range_data_raw.edge_data_in_local) +
                        *(constant_data->range_data_raw.surf_data_in_local);
         pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
         ndt.setTransformationEpsilon(0.01);
         ndt.setStepSize(0.1);
         ndt.setResolution(1.0);
         ndt.setMaximumIterations(20);

         ndt.setInputSource(input_cloud);
         ndt.setInputTarget(target_cloud);
         transform::Rigid3d initial_constraint_ = constant_data->local_pose;
         Eigen::Matrix3f inital_rotation_ =
                     initial_constraint_.rotation().toRotationMatrix().cast<float>();
         Eigen::Matrix4f initial_constraint_mat = Eigen::Matrix4f::Identity();
         initial_constraint_mat.block(0, 0, 3, 3) = inital_rotation_;
         initial_constraint_mat.block(0, 3, 3, 1) =
                            initial_constraint_.translation().cast<float>();

         pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
                                  new pcl::PointCloud<pcl::PointXYZI>());
         ndt.align(*output_cloud, initial_constraint_mat);

         Eigen::Matrix4d initial_transform = ndt.getFinalTransformation().cast<double>();

         Eigen::Matrix3d rotation_mat = initial_transform.block(0, 0, 3, 3);
         Eigen::Quaterniond rotation_estimate(rotation_mat);
         Eigen::Vector3d translation_estimate(initial_transform.block(0, 3, 3, 1));
         transform::Rigid3d transform_estimate(translation_estimate, rotation_estimate);
         //transform::Rigid3d transform_estimate;

         std::cout << "ndt result: " << transform_estimate << std::endl;
         ceres::Solver::Summary unused_summary;
         transform::Rigid3d constraint_transform;

         ceres_scan_matcher_.setKdtree(submap);
         // transform::Rigid3d constraint_estimate;
         for (int iter = 0; iter != optimization_count; ++iter)
         {
             ceres_scan_matcher_.Match(&transform_estimate,
                                      constant_data->range_data_raw.edge_data_in_local,
                                      constant_data->range_data_raw.surf_data_in_local,
                                      submap,
                                      &unused_summary);
         }
         constraint->reset(new Constraint{
             submap_id,
             node_id,
             {submap->local_pose().inverse() * transform_estimate,
              1000,
              1000},
             Constraint::INTER_SUBMAP});
   }

    // todo::
    void ConstraintBuilder::RunWhenDoneCallback(){

    Result result;
    std::unique_ptr<std::function<void(const Result&)>> callback;
   {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);

    // 将计算完的约束进行保存
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }

    // if (log_matches) {
    //   LOG(INFO) << constraints_.size() << " computations resulted in "
    //             << result.size() << " additional constraints.";
    //   LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    // }

    // 这些约束已经保存过了, 就可以删掉了
    constraints_.clear();

    callback = std::move(when_done_);
    when_done_.reset();
    //kQueueLengthMetric->Set(constraints_.size());
  }
  // 执行回调函数 HandleWorkQueue
  (*callback)(result);

}

   // todo:
   int ConstraintBuilder::GetNumFinishedNodes(){
        absl::MutexLock locker(&mutex_);
        return num_finished_nodes_;
    }
   // todo:
   void ConstraintBuilder::NotifyEndOfNode(){
        absl::MutexLock locker(&mutex_);
        CHECK(finish_node_task_ != nullptr);

        // 生成个任务: 将num_finished_nodes_自加, 记录完成约束计算节点的总个数
        finish_node_task_->SetWorkItem([this] {
        absl::MutexLock locker(&mutex_);
        ++num_finished_nodes_;
        });

        // 将这个任务传入线程池中等待执行, 由于之前添加了依赖,
        // 所以finish_node_task_一定会比计算约束更晚完成
        auto finish_node_task_handle =
        thread_pool_->Schedule(std::move(finish_node_task_));

        // move之后finish_node_task_就没有指向的地址了, 所以这里要重新初始化
        finish_node_task_ = absl::make_unique<common::Task>();
        // 设置when_done_task_依赖finish_node_task_handle
        when_done_task_->AddDependency(finish_node_task_handle);
        ++num_started_nodes_;
    }


    void ConstraintBuilder::WhenDone(
    const std::function<void(const ConstraintBuilder::Result&)>& callback) {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ == nullptr);

    // TODO(gaschler): Consider using just std::function, it can also be empty.
    // 将回调函数赋值给when_done_
    when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
    CHECK(when_done_task_ != nullptr);

    // 生成 执行when_done_的任务
    when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
    // 将任务放入线程池中等待执行
    thread_pool_->Schedule(std::move(when_done_task_));

    // when_done_task_的重新初始化
    when_done_task_ = absl::make_unique<common::Task>();
}

}
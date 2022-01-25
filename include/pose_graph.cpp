#include "pose_graph.h"

namespace mapping
{

PoseGraph::PoseGraph(std::unique_ptr<optimization::OptimizationProblem> optimization_problem,
                   common::ThreadPool *thread_pool):
                    optimization_problem_(std::move(optimization_problem)),
                    constraint_builder_(thread_pool),
                    thread_pool_(thread_pool){
             // 构造函数
}

PoseGraph::~PoseGraph(){
    WaitForAllComputations();
    absl::MutexLock locker(&work_queue_mutex_);
    CHECK(work_queue_ == nullptr);
}

std::vector<SubmapId> PoseGraph::InitializeGlobalSubmapPoses(
        int trajectory_id,
        const std::vector<std::shared_ptr<const Submap>>& insertion_submaps){
    CHECK(!insertion_submaps.empty());

    const auto &submap_data = optimization_problem_->submap_data();

    // slam刚刚启动的时候，insertion_submaps.size()为1
    if(insertion_submaps.size() == 1){
         // 如果判断submap_data的size为0, 这条轨迹上还没有添加submap的pose
         if(submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0){
             optimization_problem_->AddSubmap(trajectory_id,
                   ComputeLocalToGlobalTransform(data_.global_submap_poses, trajectory_id) *
                   insertion_submaps[0]->local_pose()
             );
         }
         CHECK_EQ(1, submap_data.SizeOfTrajectoryOrZero(trajectory_id));

         const SubmapId submap_id = {trajectory_id, 0};
         // 因为是第一个submap，所以submap的id应该是0
         CHECK(data_.submap_data.at(submap_id).submap == insertion_submaps.front());
         return {submap_id};
    }
    CHECK_EQ(2, insertion_submaps.size());

    const auto end_it = submap_data.EndOfTrajectory(trajectory_id);
    CHECK(submap_data.BeginOfTrajectory(trajectory_id) != end_it);

    const SubmapId last_submap_id = std::prev(end_it)->id;

    // 如果是等于第一个子图, 说明insertion_submaps的第二个子图还没有加入到optimization_problem_中
    // 拿着optimization_problem_中子图的索引, 根据这个索引在data_.submap_data中获取地图的指针

    if(data_.submap_data.at(last_submap_id).submap == insertion_submaps.front()){
        // 这种情况下, 要给新的submap分配id, 并把它加到OptimizationProblem的submap_data_这个容器中
        const auto &first_submap_pose = submap_data.at(last_submap_id).global_pose;

        optimization_problem_->AddSubmap(
            trajectory_id,
            first_submap_pose * (*insertion_submaps[0]).local_pose().inverse() *
            (*insertion_submaps[1]).local_pose());
        return {last_submap_id,
         SubmapId{trajectory_id, last_submap_id.submap_index + 1}};
    }

    // 如果是等于insertion_submaps的第二个元素,
    // 说明insertion_submaps的第二个元素已经分配了id, 并加入到了OptimizationProblem的submap_data_中了
    CHECK(data_.submap_data.at(last_submap_id).submap == insertion_submaps.back());
    // 那么第一个子图的index就是last_submap_id.submap_index的前一个, 所以要-1
    const SubmapId front_submap_id{trajectory_id,
                                 last_submap_id.submap_index - 1};
    CHECK(data_.submap_data.at(front_submap_id).submap == insertion_submaps.front());

    return {front_submap_id, last_submap_id};
}

transform::Rigid3d PoseGraph::GetLocalToGlobalTransform(const int trajectory_id) const{
    absl::MutexLock locker(&mutex_);
    return ComputeLocalToGlobalTransform(data_.global_submap_poses, trajectory_id);
}

transform::Rigid3d PoseGraph::ComputeLocalToGlobalTransform(
        const MapById<SubmapId, optimization::SubmapSpec>& global_submap_poses,
        const int trajectory_id) const{
    auto begin_it = global_submap_poses.BeginOfTrajectory(trajectory_id);
    auto end_it = global_submap_poses.EndOfTrajectory(trajectory_id);
    if(begin_it == end_it)
    {
         return transform::Rigid3d::Identity();
    }

    // 获取最后一个子图的id
    const SubmapId last_optimization_submap_id = std::prev(end_it)->id;

    return global_submap_poses.at(last_optimization_submap_id).global_pose *
           data_.submap_data.at(last_optimization_submap_id).submap->local_pose().inverse();
}

 NodeId PoseGraph::AppendNode(
         std::shared_ptr<const TrajectoryNode::Data> constant_data,
         const int trajectory_id,
         const std::vector<std::shared_ptr<const Submap>>& insertion_submaps,
        const transform::Rigid3d& optimized_pose){
        absl::MutexLock locker(&mutex_);
        // 向节点中添加一个新的节点，向位姿图中添加节点
        const NodeId node_id = data_.trajectory_nodes.Append(trajectory_id,
            TrajectoryNode{constant_data, optimized_pose});
        // 节点总数加1
        ++data_.num_trajectory_nodes;

        if(data_.submap_data.SizeOfTrajectoryOrZero(trajectory_id) == 0 ||
           std::prev(data_.submap_data.EndOfTrajectory(trajectory_id))
              ->data.submap != insertion_submaps.back()){
            // 如果insertion_submaps.back()是第一次看到，也就是新生成的
            // 在data_.submap_data中加入一个空的InternalSubmapData

            const SubmapId submap_id =
                data_.submap_data.Append(trajectory_id, InternalSubmapData());

            data_.submap_data.at(submap_id).submap = insertion_submaps.back();
            LOG(INFO) << "Insertion submap " << submap_id << ".";
        }
        return node_id;
 }

 NodeId PoseGraph::AddNode(
             std::shared_ptr<const TrajectoryNode::Data> constant_data,
             const std::vector<std::shared_ptr<const Submap>> &insertion_submaps){

        const transform::Rigid3d optimized_pose(
        GetLocalToGlobalTransform(trajectory_id_) * constant_data->local_pose);

        // 添加nodeid
        const NodeId node_id = AppendNode(constant_data, trajectory_id_,
                                    insertion_submaps, optimized_pose);

        const bool newly_finished_submap =
                      insertion_submaps.front()->insertion_finished();

        // 把计算约束的工作放入workitem中等待执行
       AddWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
                 return ComputeConstraintsForNode(node_id, insertion_submaps,
                                     newly_finished_submap);
       });
       return node_id;
}

void PoseGraph::AddWorkItem(
    const std::function<WorkItem::Result()>& work_item) {
  absl::MutexLock locker(&work_queue_mutex_);

  if (work_queue_ == nullptr) {
    // work_queue_的初始化
    work_queue_ = absl::make_unique<WorkQueue>();
    // 将 执行一次DrainWorkQueue()的任务 放入线程池中等待计算
    auto task = absl::make_unique<common::Task>();
    task->SetWorkItem([this]() { DrainWorkQueue(); });
    thread_pool_->Schedule(std::move(task));
  }

  const auto now = std::chrono::steady_clock::now();
  // 将传入的任务放入work_queue_队列中
  work_queue_->push_back({now, work_item});
}

void PoseGraph::DrainWorkQueue() {
  bool process_work_queue = true;
  size_t work_queue_size;

  // 循环一直执行, 直到队列为空或需要优化时退出循环
  while (process_work_queue) {
    std::function<WorkItem::Result()> work_item;
    {
      absl::MutexLock locker(&work_queue_mutex_);
      // 退出条件1 如果任务队列空了, 就将work_queue_的指针删除
      if (work_queue_->empty()) {
        work_queue_.reset();
        return;
      }
      // 取出第一个任务
      work_item = work_queue_->front().task;
      // 将取出的任务从任务队列中删掉
      work_queue_->pop_front();
      work_queue_size = work_queue_->size();
      //kWorkQueueSizeMetric->Set(work_queue_size);
    }
    // 执行任务
    // 退出条件2 执行任务后的结果是需要优化, process_work_queue为false退出循环
    process_work_queue = work_item() == WorkItem::Result::kDoNotRunOptimization;
  }

  LOG(INFO) << "Remaining work items in queue: " << work_queue_size;
  // We have to optimize again.
  // 退出循环后, 首先等待计算约束中的任务执行完, 再执行HandleWorkQueue,进行优化
  constraint_builder_.WhenDone(
      [this](const optimization::ConstraintBuilder::Result& result) {
        HandleWorkQueue(result);
      });
}

 // TODO:
 void PoseGraph::HandleWorkQueue(const std::vector<Constraint>& result){
   {
     absl::MutexLock locker(&mutex_);
     data_.constraints.insert(data_.constraints.end(), result.begin(),
                              result.end());
   }
   RunOptimization();

  //  {
  //    absl::MutexLock locker(&mutex_);
  //    for(const Constraint& constraint : result){

  //    }
  //  }
    DrainWorkQueue();
 }

 void PoseGraph::ComputeConstraint(const NodeId& node_id, const SubmapId& submap_id){

     const transform::Rigid3d global_node_pose =
      optimization_problem_->node_data().at(node_id).global_pose_3d;

     const transform::Rigid3d global_submap_pose =
      optimization_problem_->submap_data().at(submap_id).global_pose;
     bool maybe_add_local_constrain = false;
     //bool maybe_add_global_constrain = false;

     const TrajectoryNode::Data *constant_data;
     const Submap *submap;

     {
       absl::MutexLock locker(&mutex_);
       CHECK(data_.submap_data.at(submap_id).state == SubmapState::kFinished);
       // 如果是未完成的地图不进行约束的计算
       if(!data_.submap_data.at(submap_id).submap->insertion_finished()){
         return;
       }
       // 进行 局部搜索窗口 的约束计算(对局部子图进行回环检测)
       maybe_add_local_constrain = true;

       constant_data = data_.trajectory_nodes.at(node_id).constant_data.get();
       submap = static_cast<const Submap *>(
           data_.submap_data.at(submap_id).submap.get());
     }
     if(maybe_add_local_constrain){
          // TODO:
          constraint_builder_.MaybeAddConstraint(submap_id, submap, node_id,
                                                 constant_data, global_node_pose,
                                                 global_submap_pose);
     }
}

 WorkItem::Result PoseGraph::ComputeConstraintsForNode(
        const NodeId& node_id,
        std::vector<std::shared_ptr<const Submap>> insertion_submaps,
        bool newly_finished_submap){
        std::vector<SubmapId> submap_ids;           //活跃状态下的子图id
        std::vector<SubmapId> finished_submap_ids;  // 处于完成状态的子图id
        std::set<NodeId> newly_finished_submap_node_ids;

        {
            absl::MutexLock locker(&mutex_);
            // 获取节点信息数据
            const auto &constant_data =
                data_.trajectory_nodes.at(node_id).constant_data;
            submap_ids = InitializeGlobalSubmapPoses(node_id.trajectory_id,
            insertion_submaps);

            CHECK_EQ(submap_ids.size(), insertion_submaps.size());

            // 获取这两个submap中的前一个id
            const SubmapId matching_id = submap_ids.front();
            const transform::Rigid3d local_pose = constant_data->local_pose;
            const transform::Rigid3d global_pose =
                optimization_problem_->submap_data().at(matching_id).global_pose *
                insertion_submaps.front()->local_pose().inverse() * local_pose;

            optimization_problem_->AddTrajectoryNode(
                matching_id.trajectory_id,
                optimization::NodeSpec{local_pose, global_pose});

            for(size_t i = 0; i < insertion_submaps.size(); ++i){
                const SubmapId submap_id = submap_ids[i];
                CHECK(data_.submap_data.at(submap_id).state ==
                 SubmapState::kNoConstraintSearch);
                data_.submap_data.at(submap_id).node_ids.emplace(node_id);
                const transform::Rigid3d constrain_transform =
                    insertion_submaps[i]->local_pose().inverse() * local_pose;

                data_.constraints.push_back(Constraint{
                    submap_id,
                    node_id,
                    {constrain_transform, matcher_translation_weight,
                     matcher_rotation_weight},
                    Constraint::INTRA_SUBMAP});
            }

            // ?: 找到所有已经标记为kFinished状态的submap的id(不确定，不行的话重新改graph_node.h)
            for (const auto& submap_id_data : data_.submap_data)
            {
              if(submap_id_data.data.state == SubmapState::kFinished){
                CHECK_EQ(submap_id_data.data.node_ids.count(node_id), 0);
                finished_submap_ids.emplace_back(submap_id_data.id);
              }
            }

            //如果是刚刚finished的submap
            if(newly_finished_submap){
              const SubmapId newly_finished_submap_id = submap_ids.front();
              InternalSubmapData &finished_submap_data =
                  data_.submap_data.at(newly_finished_submap_id);
              // 检查他是不是kNoConstraintSearch
              CHECK(finished_submap_data.state == SubmapState::kNoConstraintSearch);
              // 把他设置成kFinished
              finished_submap_data.state = SubmapState::kFinished;
              // 刚刚结束的这个子图中包含的所有节点
              newly_finished_submap_node_ids = finished_submap_data.node_ids;
            }
        }
        // Step: 当前节点与所有已经完成的子图进行约束的计算---实际上就是回环检测
        for(const auto& submap_id : finished_submap_ids){
          // TODO:
          ComputeConstraint(node_id, submap_id);
        }

        // Step: 计算所有节点与刚完成子图间的约束---实际上就是回环检测
        if(newly_finished_submap){
          const SubmapId newly_finished_submap_id = submap_ids.front();
          for(const auto& node_id_data : optimization_problem_->node_data()){
            const NodeId& node_id = node_id_data.id;
            if(newly_finished_submap_node_ids.count(node_id) == 0){
              ComputeConstraint(node_id, newly_finished_submap_id);
            }
          }
        }

        // TODO:
        constraint_builder_.NotifyEndOfNode();

        absl::MutexLock locker(&mutex_);
        ++num_nodes_since_last_loop_closure_;

        // Step: 插入的节点数大于optimize_every_n_nodes时执行一次优化
        if(optimize_every_n_nodes > 0 &&
        num_nodes_since_last_loop_closure_ > optimize_every_n_nodes){
          return WorkItem::Result::kDoNotRunOptimization;
        }
        return WorkItem::Result::kDoNotRunOptimization;
 }


  MapById<SubmapId, PoseGraph::SubmapData> PoseGraph::GetSubmapDataUnderLock(const SubmapId submap_id) const{

  }

  std::vector<PoseGraphInterface::Constraint> PoseGraph::constraints() const{
      absl::MutexLock locker(&mutex_);
      return data_.constraints;
  }

  void PoseGraph::RunOptimization(){
     if(optimization_problem_->submap_data().size() == 0){
       return;
     }

     optimization_problem_->Solve(data_.constraints, data_.trajectories_state.state);

     absl::MutexLock locker(&mutex_);

     const auto& submap_data = optimization_problem_->submap_data();
     const auto& node_data = optimization_problem_->node_data();

     for(const auto& node : node_data.trajectory(trajectory_id_)){
       data_.trajectory_nodes.at(node.id).global_pose = node.data.global_pose_3d;
     }
     // 推断尚未包含在“optimization_problem_”中的所有点云姿势
     // TODO:暂时先不实现
  }

  void PoseGraph::WaitForAllComputations() {
    int num_trajectory_nodes;
    {
      absl::MutexLock locker(&mutex_);
      num_trajectory_nodes = data_.num_trajectory_nodes;
    }

    // TODO:
    const int num_finished_nodes_at_start =
        constraint_builder_.GetNumFinishedNodes();

    // 报告节点的计算速度
    auto report_progress = [this, num_trajectory_nodes,
                            num_finished_nodes_at_start]() {
         if(num_trajectory_nodes != num_finished_nodes_at_start){
           std::ostringstream progress_info;
           progress_info << "Optimizing: " << std::fixed << std::setprecision(1)
                         << 100. *
                                (constraint_builder_.GetNumFinishedNodes() -
                                 num_finished_nodes_at_start) /
                                (num_trajectory_nodes - num_finished_nodes_at_start)
                         << "%...";
           std::cout << "\r\x1b[k" << progress_info.str() << std::flush;
         }
    };

    {
      const auto predicate = [this]()
                                 EXCLUSIVE_LOCKS_REQUIRED(work_queue_mutex_)
      {
        return work_queue_ == nullptr;
      };
      absl::MutexLock locker(&work_queue_mutex_);

      while (!work_queue_mutex_.AwaitWithTimeout(
          absl::Condition(&predicate),
          absl::FromChrono(common::FromSeconds(1.)))) {
            report_progress();
          }
    }

    // 现在等待任何挂起的约束计算完成
  absl::MutexLock locker(&mutex_);
  bool notification = false;
  constraint_builder_.WhenDone(
      [this,
       &notification](const optimization::ConstraintBuilder::Result& result)
          LOCKS_EXCLUDED(mutex_) {
            absl::MutexLock locker(&mutex_);
            // 保存新计算的约束
            data_.constraints.insert(data_.constraints.end(), result.begin(),
                                     result.end());
            notification = true;
          });

  const auto predicate = [&notification]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return notification;
  };

  // 等待直到notification为true
  while (!mutex_.AwaitWithTimeout(absl::Condition(&predicate),
                                  absl::FromChrono(common::FromSeconds(1.)))) {
    report_progress();
  }
  CHECK_EQ(constraint_builder_.GetNumFinishedNodes(), num_trajectory_nodes);
  std::cout << "\r\x1b[KOptimizing: Done.     " << std::endl;

}

} // namespace mapping
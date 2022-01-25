
#include "trajectory_builder.h"

LocalTrajectoryBuilder::LocalTrajectoryBuilder():thread_pool_(4){
    pose_graph_ = absl::make_unique<mapping::PoseGraph>(
        absl::make_unique<optimization::OptimizationProblem>(),
        &thread_pool_);
}


LocalTrajectoryBuilder::LocalTrajectoryBuilder(const common::OdomOptions& odom_options)
     :thread_pool_(odom_options.num_thread_pool),
     fullMap(new pcl::PointCloud<pcl::PointXYZI>()),
     scan_period(odom_options.scan_period),
     speed_filter(odom_options.speed_filter){
     pose_graph_ = absl::make_unique<mapping::PoseGraph>(
        absl::make_unique<optimization::OptimizationProblem>(),
        &thread_pool_);

    downSizeFilterEdge.setLeafSize(odom_options.map_resolution,
                            odom_options.map_resolution, odom_options.map_resolution);
    downSizeFilterSurf.setLeafSize(odom_options.map_resolution * 2,
                       odom_options.map_resolution * 2, odom_options.map_resolution * 2);
    downSizeFullMap.setLeafSize(odom_options.map_resolution,
                      odom_options.map_resolution, odom_options.map_resolution);

    optimization_count = 2;

    subEdgeLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge",
       100, &LocalTrajectoryBuilder::velodyneEdgeHandler, this);
    subSurfLaserCloud =
      nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf",
       100, &LocalTrajectoryBuilder::velodyneSurfHandler, this);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    pubSubmap_corner = nh.advertise<sensor_msgs::PointCloud2>("/submap_corner", 100);
    pubSubmap_surf = nh.advertise<sensor_msgs::PointCloud2>("/submap_surf", 100);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    srvSavePcd = nh.advertiseService("save_pcd", &LocalTrajectoryBuilder::savePcdService, this);

    odom = Eigen::Isometry3d::Identity();
    last_odom = Eigen::Isometry3d::Identity();

    active_submaps_.set_num_range_data(odom_options.num_range_data);
}


void LocalTrajectoryBuilder::velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void LocalTrajectoryBuilder::velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void LocalTrajectoryBuilder::odom_estimation(){
    while(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){
            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() &&
                (pointCloudSurfBuf.front()->header.stamp.toSec() <
                    pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5 *
                      scan_period)){
                pointCloudSurfBuf.pop();
                LOG(WARNING) << "time stamp unaligned with extra point cloud,"
                        "pls check your data-- >odom correction ";
            mutex_lock.unlock();
                continue;
            }

            if(!pointCloudEdgeBuf.empty() &&
              (pointCloudEdgeBuf.front()->header.stamp.toSec() <
                 pointCloudSurfBuf.front()->header.stamp.toSec()-0.5 *
                   scan_period)){
                pointCloudEdgeBuf.pop();
                LOG(WARNING) << "time stamp unaligned with extra point cloud,"
                                "pls check your data --> odom correction";
            mutex_lock.unlock();
                continue;
            }
            //if time aligned

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);

            // 计时并输出时间
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            ROS_INFO("average odom estimation time %f ms \n \n", total_time/total_frame);

            // 发布tf变换和里程计
            publishRosTf(pointcloud_time);

            std::shared_ptr<const Submap> first_active_submap = getMap();
            pcl::PointCloud<pcl::PointXYZI>::Ptr first_active_submap_corner =
                                                    first_active_submap->conerMap();
            publishPointCloud(first_active_submap_corner,
                                         pointcloud_time, pubSubmap_corner);

            pcl::PointCloud<pcl::PointXYZI>::Ptr first_active_submap_surf
                                                   = first_active_submap->surfMap();
            publishPointCloud(first_active_submap_surf,
                                         pointcloud_time, pubSubmap_surf);

            ++num_of_pointcloud;
            // if(num_of_pointcloud % 5 == 0){
            //     fout.open("/home/xcy/txt/odom.txt", std::ios::app);
            //     fout << odom << std::endl;
            //     fout.close();
            // }
            //  if(num_of_submaps > 29){
            //      ++num_of_pointcloud;
            //      if(num_of_pointcloud % 5 == 0){
            // //     std::string corner_name = savePcdDirectory + "/Cornerpoint_" +
            // //                               std::to_string(num_of_pointcloud) + ".pcd";
            // //     std::string surf_name = savePcdDirectory + "/Surfpoint_" +
            // //                  std::to_string(num_of_pointcloud) + ".pcd";

            // //     pcl::io::savePCDFileBinary(corner_name, *pointcloud_edge_in);
            // //     pcl::io::savePCDFileBinary(surf_name, *pointcloud_surf_in);
            //      fout.open("/home/xcy/txt/local_pose1.txt", std::ios::app);
            //      fout << odom.rotation().w() << " "
            //       << odom.rotation().x() << " "
            //       << odom.rotation().y() << " "
            //       << odom.rotation().z() << " "
            //       << std::endl;
            //      fout.close();
            //      }
            //  }
            // if(first_active_submap->insertion_finished()){
            //     LOG(INFO) << "-----------full map!----------";
            //     *fullMap += *first_active_submap_corner;
            //     *fullMap += *first_active_submap_surf;

            //     downSizeFullMap.setInputCloud(fullMap);
            //     downSizeFullMap.filter(*fullMap);
            //     ++num_of_submaps;
                //test
                // if(num_of_submaps > 13){
                //     std::string corner_name = savePcdDirectory + "/CornerSubmap_" +
                //               std::to_string(num_of_submaps) + ".pcd";
                //     std::string surf_name = savePcdDirectory + "/SurfSubmap_" +
                //              std::to_string(num_of_submaps) + ".pcd";

                //     pcl::io::savePCDFileBinary(corner_name, *first_active_submap_corner);
                //     pcl::io::savePCDFileBinary(surf_name, *first_active_submap_surf);
                //     fout.open("/home/xcy/txt/local_submap_pose.txt", std::ios::app);
                //     fout << first_active_submap->local_pose() << std::endl;
                //     fout.close();
                // }
            //  }
            //  if(num_of_submaps > 13){
            //      fout2.open("/home/xcy/txt/between.txt", std::ios::app);
            //      fout2 << odom << std::endl;
            //      fout2.close();
            //  }
            //publishPointCloud(fullMap, pointcloud_time, pubMap);

        }
}

void LocalTrajectoryBuilder::updatePointsToMap(
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_in,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_in){

    if(optimization_count>2)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    transform::Rigid3d pose_estimate(odom.translation(), Eigen::Quaterniond(odom.rotation()));
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledEdgeCloud(
                                         new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledSurfCloud(
                                         new pcl::PointCloud<pcl::PointXYZI>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);
    if(active_submaps_.submaps().empty())
    {
        std::vector<std::shared_ptr<const Submap>> insertion_submaps =
                      active_submaps_.InsertRangeData(downsampledEdgeCloud,
                                                     downsampledSurfCloud, pose_estimate);
    }
    std::shared_ptr<const Submap> matching_submap =
        active_submaps_.submaps().front();
    // ROS_WARN("point nyum%d,%d",(int)downsampledEdgeCloud->points.size(), (int)downsampledSurfCloud->points.size());
    if(matching_submap->conerMap()->points.size()>10 &&
       matching_submap->surfMap()->points.size()>50){
        ceres_scan_matcher.setKdtree(matching_submap.get());
        for (int iterCount = 0; iterCount < optimization_count; iterCount++)
        {
            ceres::Solver::Summary summary;
            ceres_scan_matcher.Match(&pose_estimate, downsampledEdgeCloud, downsampledSurfCloud,
            matching_submap.get(), &summary);
        }
        std::cout << pose_estimate << std::endl;
    }
    else{
        LOG(WARNING) << "not enough points in map to associate, map error";
    }

    odom = Eigen::Isometry3d::Identity();
    odom.linear() = pose_estimate.rotation().toRotationMatrix();
    odom.translation() = pose_estimate.translation();

    //LOG(INFO) << "movement: " << (odom.translation() - last_odom.translation()).norm();

    if((odom.translation() - last_odom.translation()).norm() < 0.1)
    {
        LOG(INFO) << "------------skip---------------";
        return;
    }
    // 将原点位于local坐标系原点处的点云 变换成 原点位于匹配位姿处的点云。
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_in_local(
                                              new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_points_in_local(
                                              new pcl::PointCloud<pcl::PointXYZI>());
    edge_points_in_local->reserve(downsampledEdgeCloud->points.size());
    surf_points_in_local->reserve(downsampledSurfCloud->points.size());
    for (int i = 0; i < (int)downsampledEdgeCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        ceres_scan_matcher.TransformPointData(&(downsampledEdgeCloud
        ->points[i]), &point_temp, pose_estimate);
        edge_points_in_local->push_back(point_temp);
    }
    for (int i = 0; i < (int)downsampledSurfCloud->points.size(); i++)
    {
        pcl::PointXYZI point_temp;
        ceres_scan_matcher.TransformPointData(&(downsampledSurfCloud
        ->points[i]), &point_temp, pose_estimate);
        surf_points_in_local->push_back(point_temp);
    }

    // 将原点位于匹配位姿处的点云插入到submap中
    std::vector<std::shared_ptr<const Submap>> insertion_submaps =
        active_submaps_.InsertRangeData(edge_points_in_local,
                                         surf_points_in_local, pose_estimate);
    //std::cout << "the size of submaps is: " << insertion_submaps.size() << std::endl;
    //std::cout << "num of submaps: " <<
    //                  insertion_submaps.front()->num_range_data() << std::endl;

}


void LocalTrajectoryBuilder::downSamplingToMap(
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& edge_pc_out,
                   const pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_in,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);
}


std::shared_ptr<const Submap> LocalTrajectoryBuilder::getMap(){
    return active_submaps_.submaps().front();
}

void LocalTrajectoryBuilder::publishRosTf(const ros::Time& pointcloud_time){
     Eigen::Quaterniond q_current(odom.rotation());

     static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(odom.translation().x(),
                                    odom.translation().y(), odom.translation().z()) );
     tf::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
     transform.setRotation(q);
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

     nav_msgs::Odometry laserOdometry;
     laserOdometry.header.frame_id = "map";
     laserOdometry.child_frame_id = "base_link";
     laserOdometry.header.stamp = pointcloud_time;
     laserOdometry.pose.pose.orientation.x = q_current.x();
     laserOdometry.pose.pose.orientation.y = q_current.y();
     laserOdometry.pose.pose.orientation.z = q_current.z();
     laserOdometry.pose.pose.orientation.w = q_current.w();
     laserOdometry.pose.pose.position.x = odom.translation().x();
     laserOdometry.pose.pose.position.y = odom.translation().y();
     laserOdometry.pose.pose.position.z = odom.translation().z();
     pubLaserOdometry.publish(laserOdometry);
}

void LocalTrajectoryBuilder::publishPointCloud(
               const pcl::PointCloud<pcl::PointXYZI>::Ptr map_to_publish,
               const ros::Time& pointcloud_time,
               const ros::Publisher& pub){
    sensor_msgs::PointCloud2 publish_map;
    pcl::toROSMsg(*map_to_publish, publish_map);
    publish_map.header.frame_id = "map";
    publish_map.header.stamp = pointcloud_time;
    pub.publish(publish_map);
}

bool LocalTrajectoryBuilder::savePcdService(floam::save_pcdRequest& req, floam::save_pcdRequest& res){
    std::string savePcdDirectory;
    std::cout << "Saving pointcloud to pcd files .." << std::endl;
    savePcdDirectory = std::getenv("HOME") + req.destination;

    std::string corner_name = savePcdDirectory + "/CornerSubmap_" +
                              std::to_string(num_of_submaps) + ".pcd";
    std::string surf_name = savePcdDirectory + "/SurfSubmap_" +
                             std::to_string(num_of_submaps) + ".pcd";

    //pcl::io::savePCDFileBinary(corner_name, *first_active_submap_corner);
    //pcl::io::savePCDFileBinary(surf_name, *first_active_submap_surf);
    //fout.open("/home/xcy/txt/local_submap_pose.txt", std::ios::app);

}
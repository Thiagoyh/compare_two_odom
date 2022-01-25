
#include "scanRegistration.h"

scanRegistration::scanRegistration(common::LidarOptions &lidar_param)
                         : lidar_param_(lidar_param){
    LOG(INFO) << "lidar_process starting!! ";

    subLaserCloud = nh_.subscribe(
        "/velodyne_points", 100, &scanRegistration::velodyneHandler, this);

    pubLaserCloudFiltered = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubEdgePoints = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100);

    pubSurfPoints = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100);

}

void scanRegistration::velodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &laserCloudMsg){
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void scanRegistration::laser_processing(){
      while(!pointCloudBuf.empty()){
        {
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(
                                              new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(
                                            new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(
                                            new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(
                                                new pcl::PointCloud<pcl::PointXYZI>());
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints.publish(edgePointsMsg);


            sensor_msgs::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints.publish(surfPointsMsg);

        }
    }
}

void scanRegistration::featureExtraction(
                          const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);


    int N_SCANS = lidar_param_.num_lines;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    for(int i=0;i<N_SCANS;i++){
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(
                                            new pcl::PointCloud<pcl::PointXYZI>()));
    }

    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        int scanID=0;
        double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x +
                               pc_in->points[i].y * pc_in->points[i].y);

        if(distance < lidar_param_.min_distance ||
                    distance > lidar_param_.max_distance)
            continue;
        double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;

        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64)
        {
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
            {
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
        }
        laserCloudScans[scanID]->push_back(pc_in->points[i]);

    }

    for(int i = 0; i < N_SCANS; i++){
        if(laserCloudScans[i]->points.size()<131){
            continue;
        }

        std::vector<Double2d> cloudCurvature;
        int total_points = laserCloudScans[i]->points.size()-10;
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){

            double diffX = laserCloudScans[i]->points[j - 5].x +
             laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x +
             laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x -
             10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x +
             laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x +
              laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;

            double diffY = laserCloudScans[i]->points[j - 5].y +
            laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y +
            laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y -
            10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y +
            laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y +
            laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;

            double diffZ = laserCloudScans[i]->points[j - 5].z +
            laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z +
            laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z -
            10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z +
            laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z +
            laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;

            Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }
        for(int j=0;j<6;j++){
            int sector_length = (int)(total_points/6);
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = total_points - 1;
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin() +
                                       sector_start,cloudCurvature.begin()+sector_end);

            featureExtractionFromSector(laserCloudScans[i],subCloudCurvature,
                                                    pc_out_edge, pc_out_surf);

        }
    }
}


void scanRegistration::featureExtractionFromSector(
                           const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
                           std::vector<Double2d>& cloudCurvature,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    std::sort(cloudCurvature.begin(), cloudCurvature.end(),
                 [](const Double2d & a, const Double2d & b)
    {
        return a.value < b.value;
    });

    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id;
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.1){
                break;
            }

            largestPickedNum++;
            picked_points.push_back(ind);

            if (largestPickedNum <= 20){
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }

    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id;
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }

}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
    layer = layer_in;
    time = time_in;
};

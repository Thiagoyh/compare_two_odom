
#include <thread>

#include "scanRegistration.h"
#include "glog/logging.h"
#include "gflags/gflags.h"

DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");


int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::GLOG_INFO);
    google::ParseCommandLineFlags(&argc, &argv, true);
    ros::init(argc, argv, "main");

    ::common::NodeOptions node_options =
        ::common::LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

    scanRegistration lidarProcess(node_options.lidar_param_);
    //std::thread laser_processing_process(&LaserProcessing::laser_processing, std::ref(lidarProcess));

    //laser_processing_process.join();
    ros::Rate(8);
    while (ros::ok())
    {
        lidarProcess.laser_processing();
        ros::spinOnce();
    }

    return 0;
}


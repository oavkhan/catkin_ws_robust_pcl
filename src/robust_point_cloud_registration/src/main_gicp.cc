#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tools/utils.hpp>

#include "external_pcl_registration/gicp.h"


DEFINE_string(gicp_source_cloud_filename, "", "");
DEFINE_string(gicp_target_cloud_filename, "", "");
DEFINE_string(gicp_aligned_cloud_filename, "", "");
DEFINE_double(gicp_source_filter_size, 5, "");
DEFINE_double(gicp_target_filter_size, 0, "");
DEFINE_bool(gicp_save_aligned_cloud, false, "");
DEFINE_bool(gicp_visualize_clouds, false, "");
DEFINE_string(gicp_frame_id, "", "");
DEFINE_double(gicp_transformation_epsilon, 0.01, "");
DEFINE_int32(gicp_maximum_iterations, 100, "");
DEFINE_int32(gicp_maximum_optimizer_iterations, 100, "");
DEFINE_double(gicp_maximum_correspondence_distance, 10.0, "");
DEFINE_int32(gicp_ransac_iterations, 100, "");
DEFINE_double(gicp_ransac_outlier_rejection_threshold, 1.0, "");
DEFINE_bool(gicp_use_reciprocal_correspondence, false, "");
DEFINE_bool(gicp_use_default_parameters, true, "");
DEFINE_bool(gicp_publish_aligned_clouds, false, "");
DEFINE_bool(gicp_publish_map_clouds, false, "");
DEFINE_double(gicp_filter_leaf_size, 0.10, "");

void parseGicpParameters(GicpParameters* params) {
  params->source_cloud_filename = FLAGS_gicp_source_cloud_filename;
  params->target_cloud_filename = FLAGS_gicp_target_cloud_filename;
  params->aligned_cloud_filename = FLAGS_gicp_aligned_cloud_filename;
  params->visualize_clouds = FLAGS_gicp_visualize_clouds;
  params->save_aligned_cloud = FLAGS_gicp_save_aligned_cloud;
  params->frame_id = FLAGS_gicp_frame_id;
  params->transformation_epsilon = FLAGS_gicp_transformation_epsilon;
  params->maximum_iterations = FLAGS_gicp_maximum_iterations;
  params->maximum_optimizer_iterations = FLAGS_gicp_maximum_optimizer_iterations;
  params->ransac_iterations = FLAGS_gicp_ransac_iterations;
  params->ransac_outlier_rejection_threshold = FLAGS_gicp_ransac_outlier_rejection_threshold;
  params->use_reciprocal_correspondence = FLAGS_gicp_use_reciprocal_correspondence;
  params->use_default_parameters = FLAGS_gicp_use_default_parameters;
  params->publish_aligned_clouds = FLAGS_gicp_publish_aligned_clouds;
  params->publish_map_clouds = FLAGS_gicp_publish_map_clouds;
  params->filter_leaf_size = FLAGS_gicp_filter_leaf_size;
}

int main(int argc, char** argv) {
  
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "GICP");
  ros::NodeHandle nh;
  
  ros::Publisher target_cloud_publisher = 
    nh.advertise<sensor_msgs::PointCloud2> ("/target_cloud", 10);

  ros::Publisher aligned_cloud_publisher = 
    nh.advertise<sensor_msgs::PointCloud2> ("/aligned_cloud", 10);
  
  ros::Publisher map_cloud_publisher = 
    nh.advertise<sensor_msgs::PointCloud2> ("/map", 10);

  //ros::Time::init();

  // Load the GICP parameters.
  GicpParameters gicp_params;
  parseGicpParameters(&gicp_params);

  // Run GICP.
  Gicp gicp(gicp_params,
      target_cloud_publisher,
      aligned_cloud_publisher,
      map_cloud_publisher);
  
  ros::Subscriber source_cloud_subscriber = nh.subscribe(
      "/multisense/image_points2_color",
      10,
      &Gicp::PointCloudCallback,
      &gicp);

  ros::spin();

  return 0;
}

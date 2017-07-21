#include "external_pcl_registration/gicp.h"

#include <glog/logging.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>


Gicp::Gicp(const GicpParameters& params,
    ros::Publisher& target_cloud_publisher,
    ros::Publisher& aligned_cloud_publisher,
    ros::Publisher& map_cloud_publisher) :
  target_is_set_(false),
  params_(params),
  target_cloud_publisher_(target_cloud_publisher),
  aligned_cloud_publisher_(aligned_cloud_publisher),
  map_cloud_publisher_(map_cloud_publisher),
  source_cloud_(new pcl::PointCloud<PointType>),
  target_cloud_(new pcl::PointCloud<PointType>),
  map_cloud_(new pcl::PointCloud<PointType>) {}

void Gicp::PointCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if(!target_is_set_) {
    
    // read point cloud from ros msg
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
      << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (params_.filter_leaf_size,
        params_.filter_leaf_size, params_.filter_leaf_size);
    sor.filter (*cloud_filtered);
    
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
      << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    //fromROSMsg(*cloud_msg, *target_cloud_);
    pcl::fromPCLPointCloud2(*cloud_filtered, *target_cloud_);
    target_is_set_ = true;

    // initialize map cloud
    *map_cloud_ = *target_cloud_;
  
  } else {
    
    // read point cloud from ros msg
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    
    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
      << " data points (" << pcl::getFieldsList (*cloud) << ").";

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (params_.filter_leaf_size,
        params_.filter_leaf_size, params_.filter_leaf_size);
    sor.filter (*cloud_filtered);
    
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
      << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

    //fromROSMsg(*cloud_msg, *source_cloud_);
    pcl::fromPCLPointCloud2(*cloud_filtered, *source_cloud_);
    evaluate(source_cloud_, target_cloud_);
  }
}

void Gicp::evaluate(
    pcl::PointCloud<PointType>::Ptr source_cloud,
    pcl::PointCloud<PointType>::Ptr target_cloud) {
  CHECK(source_cloud);
  CHECK(target_cloud);

  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setInputSource(source_cloud);
  gicp.setInputTarget(target_cloud);

  if (!params_.use_default_parameters) {
    gicp.setMaxCorrespondenceDistance(params_.maximum_correspondence_distance);
    gicp.setMaximumIterations(params_.maximum_iterations);
    gicp.setMaximumOptimizerIterations(params_.maximum_optimizer_iterations);
    gicp.setRANSACIterations(params_.ransac_iterations);
    gicp.setRANSACOutlierRejectionThreshold(params_.ransac_outlier_rejection_threshold);
    gicp.setTransformationEpsilon(params_.transformation_epsilon);
    gicp.setUseReciprocalCorrespondences(params_.use_reciprocal_correspondence);
  }
  LOG(INFO) << "MaxCorrespondenceDistance: " << gicp.getMaxCorrespondenceDistance();
  LOG(INFO) << "MaximumIterations: " << gicp.getMaximumIterations();
  LOG(INFO) << "MaximumOptimizerIterations: " << gicp.getMaximumOptimizerIterations();
  LOG(INFO) << "RANSACIterations: " << gicp.getRANSACIterations();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << gicp.getRANSACOutlierRejectionThreshold();
  LOG(INFO) << "TransformationEpsilon: " << gicp.getTransformationEpsilon();
  LOG(INFO) << "MaxCorrespondenceDistance: " << gicp.getMaxCorrespondenceDistance();
  LOG(INFO) << "RANSACOutlierRejectionThreshold: " << gicp.getRANSACOutlierRejectionThreshold();
  LOG(INFO) << "UseReciprocalCorrespondences: " << gicp.getUseReciprocalCorrespondences();

  pcl::PointCloud<PointType>::Ptr aligned_source =
      boost::make_shared<pcl::PointCloud<PointType>>();
  gicp.align(*aligned_source);
  CHECK(aligned_source);
  LOG(INFO) << "Final transformation: " << std::endl << gicp.getFinalTransformation();
  if (gicp.hasConverged()) {
    
    // set the aligned source cloud as the next target cloud
    target_cloud_ = aligned_source;
    LOG(INFO) << "GICP converged." << std::endl
              << "The score is " << gicp.getFitnessScore();
  } else {
    // reinitialize
    target_is_set_ = false;
    LOG(INFO) << "GICP did not converge.";
  }


  if (params_.save_aligned_cloud) {
    std::string filename = params_.aligned_cloud_filename + 
      std::to_string(ros::Time::now().toSec()) + ".pcd";
    pcl::io::savePCDFileASCII(filename, *aligned_source);
    LOG(INFO) << "Saving aligned source cloud to: " << filename;
  }

  if (params_.visualize_clouds) {
    source_cloud->header.frame_id = params_.frame_id;
    target_cloud->header.frame_id = params_.frame_id;
    aligned_source->header.frame_id = params_.frame_id;
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer
        (new pcl::visualization::PCLVisualizer ("GICP: source(red), target(green), aligned(blue)"));
    viewer->setBackgroundColor(255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        source_cloud_handler(source_cloud, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        target_cloud_handler(target_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointType>
        aligned_source_handler(aligned_source, 0, 0, 255);
    viewer->addPointCloud<PointType>(source_cloud, source_cloud_handler, "source");
    viewer->addPointCloud<PointType>(target_cloud, target_cloud_handler, "target");
    viewer->addPointCloud<PointType>(aligned_source, aligned_source_handler, "aligned source");
    viewer->spin();
  }

  if (params_.publish_aligned_clouds) {
    // publish target cloud and aligned cloud 
    aligned_source->header.frame_id = target_cloud->header.frame_id;
    aligned_source->header.stamp = ros::Time::now().toNSec() / 1000;
    target_cloud->header.stamp = ros::Time::now().toNSec() / 1000;
    
    aligned_cloud_publisher_.publish(aligned_source);
    target_cloud_publisher_.publish(target_cloud);
    LOG(INFO) << "Publishing aligned source cloud.";
  }

  if (params_.publish_map_clouds) {
    *map_cloud_ = *map_cloud_ + *aligned_source;
    map_cloud_->header.frame_id = target_cloud_->header.frame_id;
    map_cloud_->header.stamp = ros::Time::now().toNSec() / 1000;
    map_cloud_publisher_.publish(map_cloud_);
    LOG(INFO) << "Publishing map cloud.";
  }
}


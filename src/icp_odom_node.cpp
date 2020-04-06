#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rev_icp_mapper/MatchCloudsREV.h"
#include "octomap_mot/FindConflict.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <vector>

using namespace sensor_msgs;
using namespace std;

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<PCLPoint> PCLPointCloud;

class IcpOdom
{
 public:
  IcpOdom(ros::NodeHandle nh_, ros::NodeHandle nh_private_);

 protected:
  void cloudCallBack(const PointCloud2::ConstPtr& cloud_msg);
  void publishAll(ros::Time stamp);
  void tfTransformToGeometryPose(const tf::StampedTransform& tf_trans, geometry_msgs::PoseStamped& pose);

  ros::NodeHandle nh, nh_private;
  bool latch_topic;

  ros::Subscriber cloud_sub;
  ros::Publisher merged_cloud_pub_, corrected_cloud_pub_, clustered_cloud_pub_;
  ros::ServiceClient cloud_matcher_client;
  tf::TransformListener tf_listener;

  // for path visualization
  ros::Publisher path_pub_;
  vector<geometry_msgs::PoseStamped> path_poses;

  // for cluster visulization
  pcl::PointCloud<pcl::PointXYZRGB> visClusterCloud;

  // for cloud visualization
  PCLPointCloud::Ptr map_cloud_, cloud_in_;

  // transformation for correcting odometry
  Eigen::Matrix4f worldToMap;

  tf::StampedTransform sensorToMapTf;  // map frame is world frame !!!

  int startup_drop, reading_cloud_counter, max_cloud;
  float maxOverlapToMerge;

  // parameter for cloud filtering
  //box filter
  float cloudBoxMinX_, cloudBoxMaxX_;
  float cloudBoxMinY_, cloudBoxMaxY_;
  float cloudBoxMinZ_, cloudBoxMaxZ_;
  // statistical
  int cloudNumNeighbros_;
  float cloudStdDevMul_;
  // voxel
  float cloudLeafSize_;
  // passthrough
  float cloudMinX_,cloudMaxX_;
  float cloudMinY_,cloudMaxY_;
  float cloudMinZ_,cloudMaxZ_;

  // param for clustering
  float cloudClusterTor_;
  int cloudClusterSizeMin_, cloudClusterSizeMax_;

  string world_frameId, sensor_frameId;
};


IcpOdom::IcpOdom(ros::NodeHandle nh_, ros::NodeHandle nh_private_)
  :
  nh(nh_),
  nh_private(nh_private_),
  latch_topic(false),
  // init cloud ptr
  map_cloud_{new PCLPointCloud},
  cloud_in_ {new PCLPointCloud},
  // icp param
  startup_drop(1),
  reading_cloud_counter(0),
  maxOverlapToMerge(0.9),
  max_cloud(15),
  //frame
  world_frameId("world"), sensor_frameId("velo_link"),
  // box filter
  cloudBoxMinX_{-1.4}, cloudBoxMaxX_{1.4},
  cloudBoxMinY_{-1.2}, cloudBoxMaxY_{1.0},
  cloudBoxMinZ_{-2.0}, cloudBoxMaxZ_{0.5},
  //statistical filter
  cloudNumNeighbros_{50}, cloudStdDevMul_{1.},
  // voxel grid
  cloudLeafSize_{0.5},
  // pass through
  cloudMinX_{0.5}, cloudMaxX_{40.},
  cloudMinY_{-15.}, cloudMaxY_{10.},
  cloudMinZ_{0.25}, cloudMaxZ_{7.0},
  // clustering
  cloudClusterTor_{0.02},
  cloudClusterSizeMin_{50}, cloudClusterSizeMax_{1000}
{
  // setup topic
  cloud_sub = nh.subscribe("cloud_in", 1, &IcpOdom::cloudCallBack, this);
  
  nh_private.param("latch_topic", latch_topic, latch_topic);
  merged_cloud_pub_ = nh.advertise<PointCloud2>("merged_cloud", 1, latch_topic);
  corrected_cloud_pub_ = nh.advertise<PointCloud2>("corrected_cloud", 1, latch_topic);
  clustered_cloud_pub_ = nh.advertise<PointCloud2>("clustered_cloud", 1, latch_topic);
  path_pub_ = nh.advertise<nav_msgs::Path>("sensor_path", 1);

  // subscribe to service
  ros::service::waitForService("rev_match_clouds");
  cloud_matcher_client = nh.serviceClient<rev_icp_mapper::MatchCloudsREV>("rev_match_clouds");
  ros::service::waitForService("find_conflict_srv");
  find_conflict_client = nh.serviceClient<octomap_mot::FindConflict>("find_conflict_srv");

  // get param
  nh_private.param("startup_drop", startup_drop, startup_drop);
  nh_private.param("maxOverlapToMerge", maxOverlapToMerge, maxOverlapToMerge);
  nh_private.param("world_frameId", world_frameId, world_frameId);
  nh_private.param("sensor_frameId", sensor_frameId, sensor_frameId);
  nh_private.param("max_cloud", max_cloud, max_cloud);
  // pointcloud filtering
  nh_private.param("pointcloud/box_min_x", cloudBoxMinX_, cloudBoxMinX_);
  nh_private.param("pointcloud/box_max_x", cloudBoxMaxX_, cloudBoxMaxX_);
  nh_private.param("pointcloud/box_min_y", cloudBoxMinY_, cloudBoxMinY_);
  nh_private.param("pointcloud/box_max_y", cloudBoxMaxY_, cloudBoxMaxY_);
  nh_private.param("pointcloud/num_neighbors", cloudNumNeighbros_, cloudNumNeighbros_);
  nh_private.param("pointcloud/std_dev_mul", cloudStdDevMul_, cloudStdDevMul_);
  nh_private.param("pointcloud/leaf_size", cloudLeafSize_, cloudLeafSize_);
  nh_private.param("pointcloud/min_x", cloudMinX_, cloudMinX_);
  nh_private.param("pointcloud/max_x", cloudMaxX_, cloudMaxX_);
  nh_private.param("pointcloud/min_y", cloudMinY_, cloudMinY_);
  nh_private.param("pointcloud/max_y", cloudMaxY_, cloudMaxY_);
  nh_private.param("pointcloud/min_z", cloudMinZ_, cloudMinZ_);
  nh_private.param("pointcloud/max_z", cloudMaxZ_, cloudMaxZ_);
  // clustering
  nh_private.param("pointcloud/cluster/tor", cloudClusterTor_, cloudClusterTor_);
  nh_private.param("pointcloud/cluster/size_min", cloudClusterSizeMin_, cloudClusterSizeMin_);
  nh_private.param("pointcloud/cluster/size_max", cloudClusterSizeMax_, cloudClusterSizeMax_);
}


void IcpOdom::tfTransformToGeometryPose(const tf::StampedTransform& tf_trans, geometry_msgs::PoseStamped& pose)
{
  // copy header
  pose.header.frame_id = tf_trans.frame_id_;
  pose.header.stamp = tf_trans.stamp_;

  // copy translation
  pose.pose.position.x = tf_trans.getOrigin().x();
  pose.pose.position.y = tf_trans.getOrigin().y();
  pose.pose.position.z = tf_trans.getOrigin().z();

  // copy orientation
  pose.pose.orientation.x = tf_trans.getRotation().x();
  pose.pose.orientation.y = tf_trans.getRotation().y();
  pose.pose.orientation.z = tf_trans.getRotation().z();
  pose.pose.orientation.w = tf_trans.getRotation().w();
}


void IcpOdom::cloudCallBack(const PointCloud2::ConstPtr& cloud_msg)
{
  // drop cloud at start up
  if (reading_cloud_counter < startup_drop) {
    ROS_INFO("Dropping cloud at startup");
    reading_cloud_counter++;
    return;
  }

  if (reading_cloud_counter > max_cloud + startup_drop) {  // stop processing condition
    publishAll(cloud_msg->header.stamp);
    return;
  }

  // get incoming cloud
  pcl::fromROSMsg(*cloud_msg, *cloud_in_);

  /// PREPROCESS INCOMING CLOUD
  // box filter 
  pcl::CropBox<PCLPoint> box;
  box.setMin(Eigen::Vector4f{cloudBoxMinX_, cloudBoxMinY_, cloudBoxMinZ_, 1.0});
  box.setMax(Eigen::Vector4f{cloudBoxMaxX_, cloudBoxMaxY_, cloudBoxMaxZ_, 1.0});
  box.setNegative(true);  // to filter the points inside the box
  box.setInputCloud(cloud_in_->makeShared());
  box.filter(*cloud_in_);

  // statistical filter
  pcl::StatisticalOutlierRemoval<PCLPoint> sor;
  sor.setMeanK(cloudNumNeighbros_);
  sor.setStddevMulThresh(cloudStdDevMul_);
  sor.setInputCloud(cloud_in_->makeShared());
  sor.filter(*cloud_in_);

  // // voxel filter
  pcl::VoxelGrid<PCLPoint> voxIn;
  voxIn.setLeafSize (cloudLeafSize_, cloudLeafSize_, cloudLeafSize_);
  voxIn.setInputCloud(cloud_in_->makeShared());
  voxIn.filter(*cloud_in_);

  // pass through filter
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(cloudMinX_, cloudMaxX_);
  pass_x.setInputCloud(cloud_in_->makeShared());
  pass_x.filter(*cloud_in_);

  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(cloudMinY_, cloudMaxY_);
  pass_y.setInputCloud(cloud_in_->makeShared());
  pass_y.filter(*cloud_in_);

  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(cloudMinZ_, cloudMaxZ_);
  pass_z.setInputCloud(cloud_in_->makeShared());
  pass_z.filter(*cloud_in_);

  // TRANSFORM cloud_in CURRENTLY EXPRESSED IN frame /velo_link TO frame /world
  tf::StampedTransform sensorToWorldTf;
  try {
    tf_listener.waitForTransform(world_frameId, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1));
    tf_listener.lookupTransform(world_frameId, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Can't find sensorToWorldTf due to: %s\nIgnoring the current scan.", ex.what());
    // Don't increase reading_cloud_counter to come back to this step next time.
    return;
  }
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  pcl::transformPointCloud(*cloud_in_, *cloud_in_, sensorToWorld);

  // init map_cloud
  if (reading_cloud_counter == startup_drop) {
    ROS_INFO("Init map_cloud with current scan");
    *map_cloud_ = *cloud_in_;

    ROS_INFO("Init sensorToMapTf with current sensorToWorldTf");
    sensorToMapTf = sensorToWorldTf;
    
    ROS_INFO("Init path with current sensorToMapTf");
    geometry_msgs::PoseStamped pose;
    tfTransformToGeometryPose(sensorToMapTf, pose);
    path_poses.push_back(pose);

    reading_cloud_counter++;
    return;
  }

  /// REGISTER READING CLOUD TO MAP CLOUD
  rev_icp_mapper::MatchCloudsREV srv;
  pcl::toROSMsg(*map_cloud_, srv.request.reference);
  pcl::toROSMsg(*cloud_in_, srv.request.readings);

  // init init_transform for ICP
  tf::Transform init_transform = sensorToMapTf * sensorToWorldTf.inverse();
  // init_transform.setIdentity();
  srv.request.init_transform.rotation.x = init_transform.getRotation().x();
  srv.request.init_transform.rotation.y = init_transform.getRotation().y();
  srv.request.init_transform.rotation.z = init_transform.getRotation().z();
  srv.request.init_transform.rotation.w = init_transform.getRotation().w();
  srv.request.init_transform.translation.x = init_transform.getOrigin().x();
  srv.request.init_transform.translation.y = init_transform.getOrigin().y();
  srv.request.init_transform.translation.z = init_transform.getOrigin().z();

  if (cloud_matcher_client.call(srv)) {
    ROS_INFO("Call match_clouds_services successfully");
    if (srv.response.overlap_ratio < maxOverlapToMerge) {
      // define transformation from reading cloud to map cloud
      tf::Quaternion worldToMap_rot(srv.response.transform.rotation.x, 
                                      srv.response.transform.rotation.y,
                                      srv.response.transform.rotation.z,
                                      srv.response.transform.rotation.w);

      tf::Vector3 worldToMap_trans(srv.response.transform.translation.x,
                                    srv.response.transform.translation.y,
                                    srv.response.transform.translation.z);

      tf::Transform worldToMapTf(worldToMap_rot, worldToMap_trans);
      pcl_ros::transformAsMatrix(worldToMapTf, worldToMap);

      /// CORRECT cloud_in
      pcl::transformPointCloud(*cloud_in_, *cloud_in_, worldToMap);
      
      // correct sensorToWorldTf with worldToMapTf (ICP result) to get sensorToMapTf (corrected odometry)
      tf::Transform sensorToMapTf_ = worldToMapTf * sensorToWorldTf;
      sensorToMapTf.setOrigin(sensorToMapTf_.getOrigin());
      sensorToMapTf.setRotation(sensorToMapTf_.getRotation());
      // update sensorToMapTf 's fields
      sensorToMapTf.frame_id_ = world_frameId;
      sensorToMapTf.child_frame_id_ = cloud_msg->header.frame_id;  // frame_id of the sensor frame
      sensorToMapTf.stamp_ = cloud_msg->header.stamp;

      // concatenate new sensor poses (w.r.t map or world) to path
      geometry_msgs::PoseStamped pose;
      tfTransformToGeometryPose(sensorToMapTf, pose);
      path_poses.push_back(pose);

      // concatenate reading_cloud with map_cloud_
      *map_cloud_ += *cloud_in_;

      // NECESSARY: subsample map_cloud_
      pcl::VoxelGrid<PCLPoint> voxMap;
      voxMap.setInputCloud(map_cloud_->makeShared());
      voxMap.setLeafSize(cloudLeafSize_, cloudLeafSize_, cloudLeafSize_);
      voxMap.filter(*map_cloud_);

      // drop map_cloud_ if number of point exist a threshold
      if (map_cloud_->width > 300000) {
        uint32_t num_before = map_cloud_->width;
        // transform map_cloud to sensor frame
        Eigen::Matrix4f mapToSensor;
        pcl_ros::transformAsMatrix(sensorToMapTf.inverse(), mapToSensor);
        pcl::transformPointCloud(*map_cloud_, *map_cloud_, mapToSensor);
        // pass through filter
        pcl::PassThrough<PCLPoint> passXMap;
        passXMap.setFilterFieldName("x");
        passXMap.setFilterLimits(-5, 100);
        passXMap.setInputCloud(map_cloud_->makeShared());
        passXMap.filter(*map_cloud_);
        cout<<"Dropping pointcloud has negative x in sensor frame\n";
        cout<<"\tNum pts after: "<<map_cloud_->width<<"\n";
        cout<<"\tReduced factor: "<<100.0-map_cloud_->width*100.0/num_before<<"\n";
        // transform map_cloud back to world frame
        pcl::transformPointCloud(*map_cloud_, *map_cloud_, mapToSensor.inverse());
      }

      // INVOKE find_conflict_server
      octomap_mot::FindConflict conf_srv;

      pcl::toROSMsg(*cloud_in_, conf_srv.request.incoming_cloud);
      // convert sensorToMapTf to geometry/TransformStamped
      conf_srv.request.sensorToMap.header.stamp = sensorToMapTf.stamp_;
      conf_srv.request.sensorToMap.transform.translation.x = sensorToMapTf.getOrigin().x();
      conf_srv.request.sensorToMap.transform.translation.y = sensorToMapTf.getOrigin().y();
      conf_srv.request.sensorToMap.transform.translation.z = sensorToMapTf.getOrigin().z();
      
      conf_srv.request.sensorToMap.transform.rotation.x = sensorToMapTf.getRotation().x();
      conf_srv.request.sensorToMap.transform.rotation.y = sensorToMapTf.getRotation().y();
      conf_srv.request.sensorToMap.transform.rotation.z = sensorToMapTf.getRotation().z();
      conf_srv.request.sensorToMap.transform.rotation.w = sensorToMapTf.getRotation().w();

      if (find_conflict_client.call(conf_srv)) { ROS_INFO("Call find_conflict_service successfully"); } 
      else { ROS_ERROR("Fail to call find_conflict_service"); }
    }
  }
  else {
    ROS_ERROR("Fail to call match_cloud services. Ignoring current scan");
  }
  publishAll(cloud_msg->header.stamp);
  reading_cloud_counter++;
}


void IcpOdom::publishAll(ros::Time stamp)
{ 
  // publish merge cloud
  if (merged_cloud_pub_.getNumSubscribers() > 0) {
    PointCloud2 map_cloud_msg;
    pcl::toROSMsg(*map_cloud_, map_cloud_msg);
    map_cloud_msg.header.frame_id = world_frameId;
    map_cloud_msg.header.stamp = stamp;
    merged_cloud_pub_.publish(map_cloud_msg);
  }

  if (clustered_cloud_pub_.getNumSubscribers() > 0) {
    // DOING: cluster cloud_in_
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree{new pcl::search::KdTree<PCLPoint>};
    tree->setInputCloud(cloud_in_->makeShared());
    
    vector<pcl::PointIndices> cluster_indices;  // each element of this vector stores indices of a cluster

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(cloudClusterTor_); 
    ece.setMinClusterSize(cloudClusterSizeMin_);
    ece.setMaxClusterSize(cloudClusterSizeMax_);
    ece.setSearchMethod (tree);
    ece.setInputCloud (cloud_in_->makeShared());
    ece.extract (cluster_indices);
    
    cout<<"[INFO] Number of found cluster: "<<cluster_indices.size()<<"\n";
    cout<<"-------------------------------------------------------------\n";
    
    // visualize cluster in cloud_in_
    int cluster_idx{0};
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered{new pcl::PointCloud<pcl::PointXYZRGB>};
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {  // iterate clusters
      uint8_t r{0}, g{0}, b{0};
      switch (cluster_idx%6) {
        case 0: r = 255; break;
        case 1: g = 255; break;
        case 2: b = 255; break;
        case 3: r = 255; g = 255; break;
        case 4: r = 255; b = 255; break;
        case 5: b = 255; g = 255; break;
      }
      for (vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit) {  // iterate points in each cluster
        pcl::PointXYZRGB p{r, g, b};
        p.x = cloud_in_->points[*pit].x;
        p.y = cloud_in_->points[*pit].y;
        p.z = cloud_in_->points[*pit].z;
        cloud_clustered->points.push_back(p);
      }
      ++cluster_idx;
    }
    // finish cloud_clustered
    cloud_clustered->height = 1;
    cloud_clustered->width = cloud_clustered->points.size();

    // publish clustered cloud
    PointCloud2 clustered_cloud_msg;
    pcl::toROSMsg(*cloud_clustered, clustered_cloud_msg);
    clustered_cloud_msg.header.frame_id = world_frameId;
    clustered_cloud_msg.header.stamp = stamp;
    clustered_cloud_pub_.publish(clustered_cloud_msg);
  }

  // publish corrected cloud
  if (corrected_cloud_pub_.getNumSubscribers() > 0) {
    PointCloud2 corrected_cloud_msg;
    pcl::toROSMsg(*cloud_in_, corrected_cloud_msg);
    corrected_cloud_msg.header.frame_id = world_frameId;
    corrected_cloud_msg.header.stamp = stamp;
    corrected_cloud_pub_.publish(corrected_cloud_msg);
  }

  // publish path
  if (path_pub_.getNumSubscribers() > 0) {
    nav_msgs::Path path;
    path.header.frame_id = world_frameId;
    path.header.stamp = stamp;
    path.poses = path_poses;
    path_pub_.publish(path);
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp_odom_node");
  ros::NodeHandle nh_, nh_private_("~");
  IcpOdom icp_odom(nh_, nh_private_);
  ros::spin();
  return 0;
}
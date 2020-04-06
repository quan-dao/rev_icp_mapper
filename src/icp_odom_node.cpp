#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rev_icp_mapper/MatchCloudsREV.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
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

  ros::Subscriber cloud_sub;
  ros::Publisher merged_cloud_pub;
  ros::ServiceClient cloud_matcher_client;
  tf::TransformListener tf_listener;

  // for path visualization
  ros::Publisher path_pub;
  vector<geometry_msgs::PoseStamped> path_poses;

  PCLPointCloud map_cloud, reading_cloud, corrected_cloud;
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

  string world_frameId, sensor_frameId;
};


IcpOdom::IcpOdom(ros::NodeHandle nh_, ros::NodeHandle nh_private_)
  :nh(nh_),
  nh_private(nh_private_),
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
  cloudMinZ_{0.25}, cloudMaxZ_{7.0}
{
  // setup topic
  cloud_sub = nh.subscribe("cloud_in", 1, &IcpOdom::cloudCallBack, this);
  
  merged_cloud_pub = nh.advertise<PointCloud2>("merged_cloud", 1);
  path_pub = nh.advertise<nav_msgs::Path>("sensor_path", 1);

  ros::service::waitForService("rev_match_clouds");
  cloud_matcher_client = nh.serviceClient<rev_icp_mapper::MatchCloudsREV>("rev_match_clouds");

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

  /// PREPROCESS INCOMING CLOUD
  PCLPointCloud cloud_in;
  pcl::fromROSMsg(*cloud_msg, cloud_in);

  // box filter 
  pcl::CropBox<PCLPoint> box;
  box.setMin(Eigen::Vector4f{cloudBoxMinX_, cloudBoxMinY_, cloudBoxMinZ_, 1.0});
  box.setMax(Eigen::Vector4f{cloudBoxMaxX_, cloudBoxMaxY_, cloudBoxMaxZ_, 1.0});
  box.setNegative(true);  // to filter the points inside the box
  box.setInputCloud(cloud_in.makeShared());
  box.filter(cloud_in);

  // statistical filter
  pcl::StatisticalOutlierRemoval<PCLPoint> sor;
  sor.setInputCloud(cloud_in.makeShared());
  sor.setMeanK(cloudNumNeighbros_);
  sor.setStddevMulThresh(cloudStdDevMul_);
  sor.filter(cloud_in);

  // // voxel filter
  pcl::VoxelGrid<PCLPoint> voxIn;
  voxIn.setInputCloud(cloud_in.makeShared());
  voxIn.setLeafSize (cloudLeafSize_, cloudLeafSize_, cloudLeafSize_);
  voxIn.filter(cloud_in);

  // pass through filter
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(cloudMinX_, cloudMaxX_);
  pass_x.setInputCloud(cloud_in.makeShared());
  pass_x.filter(cloud_in);

  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(cloudMinY_, cloudMaxY_);
  pass_y.setInputCloud(cloud_in.makeShared());
  pass_y.filter(cloud_in);

  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(cloudMinZ_, cloudMaxZ_);
  pass_z.setInputCloud(cloud_in.makeShared());
  pass_z.filter(cloud_in);

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
  pcl::transformPointCloud(cloud_in, cloud_in, sensorToWorld);

  // init map_cloud
  if (reading_cloud_counter == startup_drop) {
    ROS_INFO("Init map_cloud with current scan");
    map_cloud = cloud_in;

    ROS_INFO("Init sensorToMapTf with current sensorToWorldTf");
    sensorToMapTf = sensorToWorldTf;
    
    ROS_INFO("Init path with current sensorToMapTf");
    geometry_msgs::PoseStamped pose_;
    tfTransformToGeometryPose(sensorToMapTf, pose_);
    path_poses.push_back(pose_);

    reading_cloud_counter++;
    return;
  }

  if (reading_cloud_counter > max_cloud + startup_drop) {
    publishAll(cloud_msg->header.stamp);
    return;
  }

  /// REGISTER READING CLOUD TO MAP CLOUD
  rev_icp_mapper::MatchCloudsREV srv;
  PointCloud2 map_cloud_msg, reading_cloud_msg;
  pcl::toROSMsg(map_cloud, map_cloud_msg);
  pcl::toROSMsg(cloud_in, reading_cloud_msg);
  srv.request.reference = map_cloud_msg;
  srv.request.readings = reading_cloud_msg;

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

      pcl::transformPointCloud(cloud_in, cloud_in, worldToMap);

      // correct sensorToWorldTf with worldToMapTf (ICP result) to get sensorToMapTf (corrected odometry)
      tf::Transform sensorToMapTf_ = worldToMapTf * sensorToWorldTf;
      sensorToMapTf.setOrigin(sensorToMapTf_.getOrigin());
      sensorToMapTf.setRotation(sensorToMapTf_.getRotation());
      // update sensorToMapTf 's fields
      sensorToMapTf.frame_id_ = world_frameId;
      sensorToMapTf.child_frame_id_ = cloud_msg->header.frame_id;  // frame_id of the sensor frame
      sensorToMapTf.stamp_ = cloud_msg->header.stamp;

      // concatenate new sensor poses (w.r.t map or world) to path
      geometry_msgs::PoseStamped pose_;
      tfTransformToGeometryPose(sensorToMapTf, pose_);
      path_poses.push_back(pose_);

      // concatenate reading_cloud with map_cloud
      map_cloud += cloud_in;

      // NECESSARY: subsample map_cloud
      // uint32_t num_before(map_cloud.width);
      
      pcl::VoxelGrid<PCLPoint> vox;
      vox.setInputCloud(map_cloud.makeShared());
      vox.setLeafSize(cloudLeafSize_, cloudLeafSize_, cloudLeafSize_);
      vox.filter(map_cloud);

      // cout<<"Downsampling map cloud\n\tNum pts before: "<<num_before<<"\n";
      // cout<<"\tNum pts after: "<<map_cloud.width<<"\n";
      // cout<<"\tReduced factor: "<<100.0-map_cloud.width*100.0/num_before<<"\n";

      // drop map_cloud if number of point exist a threshold
      if (map_cloud.width>300000) {
        uint32_t num_before = map_cloud.width;
        // transform map_cloud to sensor frame
        Eigen::Matrix4f mapToSensor;
        pcl_ros::transformAsMatrix(sensorToMapTf.inverse(), mapToSensor);
        pcl::transformPointCloud(map_cloud, map_cloud, mapToSensor);
        // pass through filter
        pcl::PassThrough<PCLPoint> passXMap;
        passXMap.setFilterFieldName("x");
        passXMap.setFilterLimits(-5, 100);
        passXMap.setInputCloud(map_cloud.makeShared());
        passXMap.filter(map_cloud);
        cout<<"Dropping pointcloud has negative x in sensor frame\n";
        cout<<"\tNum pts after: "<<map_cloud.width<<"\n";
        cout<<"\tReduced factor: "<<100.0-map_cloud.width*100.0/num_before<<"\n";
        // transform map_cloud back to world frame
        pcl::transformPointCloud(map_cloud, map_cloud, mapToSensor.inverse());
      }
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
  PointCloud2 map_cloud_msg;
  pcl::toROSMsg(map_cloud, map_cloud_msg);
  map_cloud_msg.header.frame_id = world_frameId;
  map_cloud_msg.header.stamp = stamp;
  merged_cloud_pub.publish(map_cloud_msg);

  // publish path
  nav_msgs::Path path_;
  path_.header.frame_id = world_frameId;
  path_.header.stamp = stamp;
  path_.poses = path_poses;
  path_pub.publish(path_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "icp_odom_node");
  ros::NodeHandle nh_, nh_private_("~");
  IcpOdom icp_odom(nh_, nh_private_);
  ros::spin();
  return 0;
}
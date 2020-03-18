#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rev_icp_mapper/MatchCloudsREV.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
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

  string world_frameId, sensor_frameId;
};


IcpOdom::IcpOdom(ros::NodeHandle nh_, ros::NodeHandle nh_private_)
  :nh(nh_),
  nh_private(nh_private_),
  startup_drop(1),
  reading_cloud_counter(0),
  maxOverlapToMerge(0.9),
  max_cloud(15),
  world_frameId("world"),
  sensor_frameId("velo_link")
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

  PCLPointCloud cloud_in;
  pcl::fromROSMsg(*cloud_msg, cloud_in);
  // transform map_cloud currently express in /velo_link to /world frame
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


  /// Register reading cloud to map cloud
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

      // subsample map_cloud
      uint32_t num_before(map_cloud.width);
      
      pcl::VoxelGrid<PCLPoint> sor;
      sor.setInputCloud (map_cloud.makeShared());
      sor.setLeafSize (0.125f, 0.125f, 0.125f);
      sor.filter (map_cloud);

      cout<<"Downsampling map cloud\n\tNum pts before: "<<num_before<<"\n";
      cout<<"\tNum pts after: "<<map_cloud.width<<"\n";
      cout<<"\tReduced factor: "<<100.0-map_cloud.width*100.0/num_before<<"\n";
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
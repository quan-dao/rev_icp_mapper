#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "rev_icp_mapper/MatchCloudsREV.h"

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace sensor_msgs;

typedef pcl::PointXYZ PCLPoint;
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;

class CorrectedCloudPublihser
{
public:
  CorrectedCloudPublihser(ros::NodeHandle nh_, ros::NodeHandle nh_private_) : nh(nh_), nh_private(nh_private_), startup_dropout(0), reading_gap(10)
  {
    cloud_sub = nh.subscribe("/kitti/velo/pointcloud", 1, &CorrectedCloudPublihser::cloudCallBack, this);
    readingCloud_pub = nh.advertise<PointCloud2>("reading_cloud", 1);
    referenceCloud_pub = nh.advertise<PointCloud2>("reference_cloud", 1);
    correctedReadingCloud_pub = nh.advertise<PointCloud2>("corrected_reading_cloud", 1);

    matcher_client = nh.serviceClient<rev_icp_mapper::MatchCloudsREV>("match_clouds");

    nh_private.param("startup_dropout", startup_dropout, startup_dropout);
    nh_private.param("reading_gap", reading_gap, reading_gap);

  }

  void cloudCallBack(const PointCloud2::ConstPtr& cloud_in) {
    // drop cloud at start up
    if (cloud_counter < startup_dropout) {
      cloud_counter++;
      return;
    }

    if (cloud_counter == startup_dropout) {
      // save cloud in as reference cloud
      reference_cloud = *cloud_in;
      ROS_INFO("Got reference cloud");
      cloud_counter++;
      publishAll();
      return;
    }
    
    if (cloud_counter < (startup_dropout + reading_gap)) {
      cloud_counter++;
      publishAll();
      return;
    }

    if (cloud_counter == (startup_dropout + reading_gap)) {
      // save cloud in as reading cloud
      reading_cloud = *cloud_in;
      ROS_INFO("Got reading cloud");
      publishAll();
      cloud_counter++;

      /// compute corrected cloud
      rev_icp_mapper::MatchCloudsREV srv;
      srv.request.reference = reference_cloud;
      srv.request.readings = reading_cloud;
      if (matcher_client.call(srv)) {
        ROS_INFO("Call match_clouds_services successfully");
        ROS_INFO_STREAM("Resuted transformation\n\t"<<srv.response.transform);
        ROS_INFO_STREAM("Overlap reatio: "<<srv.response.overlap_ratio);
        
        // define transformation from reading to ref
        tf::Quaternion readingToRef_rot(srv.response.transform.rotation.x, 
                                        srv.response.transform.rotation.y,
                                        srv.response.transform.rotation.z,
                                        srv.response.transform.rotation.w);

        tf::Vector3 readingToRef_trans(srv.response.transform.translation.x,
                                      srv.response.transform.translation.y,
                                      srv.response.transform.translation.z);

        tf::Transform readingToRef_tf(readingToRef_rot, readingToRef_trans);

        Eigen::Matrix4f readingToRef;
        pcl_ros::transformAsMatrix(readingToRef_tf, readingToRef);

        // init corrected cloud
        pcl::PointCloud<pcl::PointXYZ> corrected_pc;
        pcl::fromROSMsg(reading_cloud, corrected_pc);
        
        // transfrom reading to ref frame
        pcl::transformPointCloud(corrected_pc, corrected_pc, readingToRef);
        
        // add color
        pcl::PointCloud<pcl::PointXYZRGB> vis_corrected_pc;
        vis_corrected_pc.header = corrected_pc.header;
        vis_corrected_pc.height = 1;
        vis_corrected_pc.width = corrected_pc.points.size();
        vis_corrected_pc.points.resize(vis_corrected_pc.width);
        for (size_t i=0; i < vis_corrected_pc.width; i++) {
          vis_corrected_pc.points[i].x = corrected_pc.points[i].x;
          vis_corrected_pc.points[i].y = corrected_pc.points[i].y;
          vis_corrected_pc.points[i].z = corrected_pc.points[i].z;
          vis_corrected_pc.points[i].r = 255;
          vis_corrected_pc.points[i].g = 0;
          vis_corrected_pc.points[i].b = 0;
        }

        // convert pcl to to ROS Msg
        pcl::toROSMsg(vis_corrected_pc, corrected_cloud);
      } else {
        ROS_ERROR("Fail to call match_cloud_services\n");
      }
    }
    publishAll();
    return;
  }


  void publishAll() {
    if (!reference_cloud.data.empty()) 
      referenceCloud_pub.publish(reference_cloud);
    
    if (!reading_cloud.data.empty())
      readingCloud_pub.publish(reading_cloud);

    if (!corrected_cloud.data.empty())
      correctedReadingCloud_pub.publish(corrected_cloud);

  }

protected:
  ros::NodeHandle nh, nh_private;
  ros::Subscriber cloud_sub;
  ros::Publisher readingCloud_pub, referenceCloud_pub, correctedReadingCloud_pub;
  ros::ServiceClient matcher_client;
  PointCloud2 reference_cloud, reading_cloud, corrected_cloud; 
  int startup_dropout, reading_gap;
  int cloud_counter;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "corrected_clouds_publisher");
  ros::NodeHandle nh_, nh_private_("~");
  CorrectedCloudPublihser corrected_cloud_pub(nh_, nh_private_);
  ros::spin();
  return 0;
}
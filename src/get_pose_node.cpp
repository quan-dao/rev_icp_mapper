#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>


class PoseRetriever 
{
public:
  PoseRetriever(ros::NodeHandle nh_, ros::NodeHandle nh_pri_) : nh(nh_), nh_private(nh_pri_), cloud_counter (0), startup_drop(1)
  {
    cloud_sub = nh.subscribe("/kitti/velo/pointcloud", 1, &PoseRetriever::cloudCallBack, this);  
    ROS_INFO("Pose retriever initialized, Waiting for bag to be played.\n");
    nh_private.param("startup_drop", startup_drop, startup_drop);
  }

  void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
    if ((cloud_counter<startup_drop) || (cloud_counter>startup_drop)) {
      cloud_counter++;
      return;
    }

    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform("world", "velo_link", cloud_msg->header.stamp, ros::Duration(0.1));
      tf_listener.lookupTransform("world", "velo_link", cloud_msg->header.stamp, transform);
    } catch (tf::TransformException ex) {
      ROS_ERROR("Can't find transform due to %s", ex.what());
      cloud_counter++;
      return;
    }
    std::cout<<"Pose of first /velo_link:\n";
    std::cout<<"\tRotation:\n"<<"\t\tx:"<<transform.getRotation().x()<<"\n"<<"\t\ty:"<<transform.getRotation().y()<<"\n"<<"\t\tz:"<<transform.getRotation().z()<<"\n"<<"\t\tw:"<<transform.getRotation().w()<<"\n";
    std::cout<<"\tTranslation:\n"<<"\t\tx:"<<transform.getOrigin().x()<<"\n"<<"\t\ty:"<<transform.getOrigin().y()<<"\n"<<"\t\tz:"<<transform.getOrigin().z()<<"\n";
    cloud_counter++;
  }

protected:
  ros::NodeHandle nh, nh_private;
  ros::Subscriber cloud_sub;
  tf::TransformListener tf_listener;
  int cloud_counter;
  int startup_drop;
};



int main(int argc, char** argv) {
  ros::init(argc, argv, "first_pose_finder");
  
  ros::NodeHandle nh, nh_pri("~");
  PoseRetriever pose_retriver(nh, nh_pri);
  ros::spin();

}
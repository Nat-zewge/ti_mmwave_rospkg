#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>

namespace pcl_to_pose{

    class PcltoPose{
        private:
            ros::NodeHandle nh_;
            ros::Subscriber pcl_subscriber;
            ros::Publisher filtered_pub;
            ros::Publisher person_pose_pub;
            tf::TransformListener tf_listener;
            tf::StampedTransform transform;
            bool has_transform;

            geometry_msgs::PoseArray people_poses;

            std::string base_link;

            pcl::visualization::CloudViewer viewer;

            public:

            PcltoPose(bool pcl_filler):
             nh_("~"),viewer("cloud viewer"),
             has_transform(false)
             {
                 base_link = "/base_radar_link";
                 people_poses.header.stamp=ros::Time::now();
                 people_poses.header.frame_id = base_link;


                 //point cloud subscriber
                 pcl_subscriber = nh_.subscribe("/radar/RScan",1,&PcltoPose::pclCallback,this)
                 filtered_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);
                 //plane_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("plane_output", 1);
                person_pose_pub=nh.advertise<geometry_msgs::PoseArray>("/",1,true)
                
             
             }























    }





}
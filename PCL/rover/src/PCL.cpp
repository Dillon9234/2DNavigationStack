#include <ros/ros.h>
#include "rover/pcl_obstacle_data.h"
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h> 
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>

class PCL{
    private:
        ros::Publisher pub,filteredPCLPublisher;
        ros::NodeHandle nh;
        bool front = false;
        bool left = false;
        bool right = false;
        double frontThreshold = 1.4;
        double frontSideThreshold = 1.0;
        double sideThreshold = 0.6;
        double sideThresholdFront = 0.5;
        const double leafSize = 0.1;
        rover::pcl_obstacle_data output;

        void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);

            // Apply Voxel Grid filtering to downsample the point cloud
            pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
            voxel_grid.setInputCloud(cloud);
            voxel_grid.setLeafSize(leafSize, leafSize, leafSize);
            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            voxel_grid.filter(*filtered_cloud);

            // Perform RANSAC segmentation to separate ground plane
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.03);
            seg.setAxis(Eigen::Vector3f(0, 0, 1));
            seg.setInputCloud(filtered_cloud);
            seg.segment(*inliers, *coefficients);

            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(filtered_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*obstacles_cloud);
            
            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(*obstacles_cloud,out);
            filteredPCLPublisher.publish(out);
            front = false;
            left = false;
            right = false;
            for (const auto& point : obstacles_cloud->points) {
                if (point.z < frontSideThreshold) {
                    if (point.x < -sideThreshold)
                        left = true;
                    else if (point.x > sideThreshold)
                        right = true;
                }
                if(point.z < frontThreshold && abs(point.x) < sideThresholdFront)
                    front = true;
                
            }
            output.front = front;
            output.left = left;
            output.right = right;
            pub.publish(output);
            
    }
        
    public:
        PCL(){
            ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/zed2/depth/points", 1, &PCL::pointCloudCallback,this);
            pub = nh.advertise<rover::pcl_obstacle_data> ("/pcl_data", 10);
            filteredPCLPublisher = nh.advertise<sensor_msgs::PointCloud2>("/obstacles",10);
            ros::Rate loop_rate(50);
            while (ros::ok()){
                ros::spinOnce(); 
                ROS_INFO("%d %d %d",front,left,right);
                loop_rate.sleep();
            }
        }
    };

int main (int argc, char** argv){
    ros::init (argc, argv, "pcl");
    PCL pcl;
    return 0;
}




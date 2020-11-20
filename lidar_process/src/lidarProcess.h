#ifndef __LIDARPROCESS_H_
#define __LIDARPROCESS_H_

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

#include <auto_driver_msgs/Obstacle.h>
#include <auto_driver_msgs/Obstacles.h>


class LidarProcess{
public:
	LidarProcess();
	~LidarProcess(){}

private:
	/* functions relative to the ros */
	void pointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& rosPointCloud2 );

	/* variables of the ros */
	ros::NodeHandle nh;
	ros::Publisher cloud_pub;
	ros::Subscriber pointCloudSub;
	
	ros::Publisher obstaclesPub_;
	
	/* parameters of the lidar process ROS node */
	std::string pointcloudSubTopic_;
	std::string obstaclePubTopic_;

	/* parameters of the CropBox filter of the pcl */
	float cropXmin_;
	float cropXmax_;
	float cropYmin_;
	float cropYmax_;
	float cropZmin_;
	float cropZmax_;
	
	/* parameters of the DownSampling filter */
	pcl::VoxelGrid<pcl::PointXYZI> vg;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_downsampled { new pcl::PointCloud<pcl::PointXYZI>{}};

	
	/* PCL source cloud data converted from sensor_msgs::PointCloud2 */
	pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloud { new pcl::PointCloud<pcl::PointXYZI>{} } ;
	
	/* variable of the PassThrough filter */
	pcl::PassThrough<pcl::PointXYZI> pass;
	
	/* variable of the CropBox fiter */
        pcl::PointCloud<pcl::PointXYZI>::Ptr CroppedCloud { new pcl::PointCloud<pcl::PointXYZI>{} } ;
	pcl::CropBox<pcl::PointXYZI> cropBox;

	/* ground filtering */
	pcl::PointCloud<pcl::PointXYZI>::Ptr ground_filtered { new pcl::PointCloud<pcl::PointXYZI>{} } ;
	pcl::ModelCoefficients::Ptr coefficients { new pcl::ModelCoefficients{} };
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	
	pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
	
	/* Euclidean Clustering */
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree { new pcl::search::KdTree<pcl::PointXYZI> {} };
	std::vector<pcl::PointCloud<pcl::PointXYZI>> clusters;// storage the clusters in a vector
        std::vector<pcl::PointIndices> clusterIndices; // create the indices of the clusters
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;// create a object of the Euclidean class

};

LidarProcess::LidarProcess()
{
	/* get the parameters of the ROS node from the parameter-server */
        nh.param<std::string>("pointcloud_sub_topic", pointcloudSubTopic_, "/lslidar_point_cloud");
        nh.param<std::string>("obstacle_pub_topic", obstaclePubTopic_, "/lidar/obstacles");
	nh.param<std::string>("obstacle_pub_topic", obstaclePubTopic_, "/lidar/obstacles");

	
        /* init the of subscriber and publisher */
        pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>( "/FrontLidar/lslidar_point_cloud", 1, &LidarProcess::pointCloudCallback, this );
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>( "/ground_filtered", 1 );
	obstaclesPub_ = nh.advertise<auto_driver_msgs::Obstacles>(obstaclePubTopic_, 1);


	/* initial the of DownSampling filter */
	vg.setLeafSize( 0.4f,0.4f, 0.4f );
	
	/* initial the of PassThrough filter */
	pass.setFilterFieldName( "intensity" );
        pass.setFilterLimits( 0, 50 );
        pass.setNegative( true );

	/* initial of the CropBox filter */
	cropBox.setMin( Eigen::Vector4f( 0, -20, -30, 1.0f ) );
        cropBox.setMax(  Eigen::Vector4f( 50, 20, 30, 1.0f ) );
	cropBox.setNegative( false );
	
	/* initial of the ground segmentation */
	seg.setOptimizeCoefficients( true );
        seg.setModelType( pcl::SACMODEL_PLANE );
        seg.setMethodType( pcl::SAC_RANSAC );
        seg.setMaxIterations( 100 );
        seg.setDistanceThreshold( 1 );
	
	/*  Euclidean Clustering  */
	ec.setClusterTolerance( 0.5 ); //clusterTolerance_
        ec.setMinClusterSize( 10 ); // minSize_
        ec.setMaxClusterSize( 1000 ); //maxSize_

}


void LidarProcess::pointCloudCallback( const sensor_msgs::PointCloud2::ConstPtr& rosPointCloud2 )
{
	ROS_INFO( "callback ..................." );
	/* convert to PCL message type */
        pcl::fromROSMsg( *rosPointCloud2, *sourceCloud );

	/* DownSampling */
	vg.setInputCloud( sourceCloud );
	vg.filter( *cloud_downsampled );
	
	/* PassThrough filtering */
	pass.setInputCloud( cloud_downsampled );
	pass.filter( *cloud_downsampled );
	
	/* CropBox filtering */
	cropBox.setInputCloud( cloud_downsampled );
	cropBox.filter( *CroppedCloud );

	/* ground segmentation */
	pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
	seg.setInputCloud( CroppedCloud );
        seg.segment( *inliers, *coefficients );
	
	if( inliers->indices.size() == 0 ){
                //ROS_INFO( "Could not estimate a planar model for the given dataset" );
                ROS_INFO( "Could not found any inliers ..." );
        }
	/* extract the ground planar */
        extract_ground.setInputCloud( CroppedCloud );
        extract_ground.setIndices( inliers );
        extract_ground.filter( *ground_filtered );
	
	/* extract the objects on the ground */
        extract_ground.setNegative( true );
        extract_ground.filter( *ground_filtered );
	
	/* Euclidean Clustering */
	tree->setInputCloud( ground_filtered );
	ec.setSearchMethod( tree );
        ec.setInputCloud( ground_filtered );
        ec.extract( clusterIndices );

	for( pcl::PointIndices getIndices : clusterIndices ){
                pcl::PointCloud<pcl::PointXYZI> cloudCluster;
                // for each point indice in each cluster
                for( int index : getIndices.indices ){
                        cloudCluster.push_back( ground_filtered->points[index] );
                }
                cloudCluster.width = cloudCluster.size();
                cloudCluster.height = 1;
                cloudCluster.is_dense = true;
                clusters.push_back( cloudCluster );
        }

	auto_driver_msgs::Obstacles obstacles;
        for( auto it = clusters.begin(); it != clusters.end(); it ++ ){
                Eigen::Vector4f min, max;
                pcl::getMinMax3D( *it, min, max );
                auto_driver_msgs::Obstacle obstacle;
                obstacle.CX = 0.5*(min[0] + max[0]);
                obstacle.CY = 0.5*(min[1] + max[1]);
                obstacle.CZ = 0.5*(min[2] + max[2]);
                obstacle.LengthX = max[0] - min[0];
                obstacle.LengthY = max[1] - min[1];
                obstacle.LengthZ = max[2] - min[2];
                obstacle.PointNum = it->size();
                obstacles.Obstacles.push_back( obstacle );
        }
        obstaclesPub_.publish( obstacles );

	
	/* convert to ROS message type */
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg( *ground_filtered, output );
        output.header.frame_id = "front_laser_link";

        /* publish the data */
        cloud_pub.publish( output );

}














#endif

























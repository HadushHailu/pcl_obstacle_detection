#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_obstacle_detection/ObjectPoint.h>

#include <vector>
#include <math.h>


class ObstacleCluster{
    private:
	ros::NodeHandle nh_;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	ros::Publisher pub_scan_, pub_cloud_,pub_markerArray_,pub_ws_, pub_objPoint_;
	ros::Subscriber sub_;
	laser_geometry::LaserProjection projector_;
        tf::TransformListener listener_;
        int cluster_size = 0;
    public:
	ObstacleCluster(){
		pub_cloud_ = nh_.advertise<PointCloud> ("/cluster_pointcloud2", 1);
		pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_scan",100);
                pub_markerArray_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
		pub_ws_ = nh_.advertise<visualization_msgs::Marker>("work_space", 10);
		pub_objPoint_ = nh_.advertise<pcl_obstacle_detection::ObjectPoint>("/cluster",100);
		sub_ = nh_.subscribe("/scan", 100, &ObstacleCluster::scanCallback, this);
	}

	void run_it(){
		visualization_msgs::Marker marker;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.id = 1;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.color.a = 0.3; // Don't forget to set the alpha!
                marker.color.r = 0.5;
                marker.color.g = 0.4;
                marker.color.b = 0.5;

                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 1;

                marker.scale.x = 3.5 ;
                marker.scale.y = 3.5 ;
                marker.scale.z = 0.1;


 	 	ros::Rate loop_rate(4);
  		while (nh_.ok())
  		{
    			pub_ws_.publish (marker);
    			ros::spinOnce ();
    			loop_rate.sleep ();
  		}

	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
		sensor_msgs::PointCloud2 sensor_cloud;

                //change scan to map frame
               if(!listener_.waitForTransform(
        		scan_in->header.frame_id,
        		"map",
        		scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
       			 ros::Duration(1.0))){
     			return;
  		}
		//change LaserScan msg to PointCloud
		//projector_.projectLaser(*scan_in, sensor_cloud);
                projector_.transformLaserScanToPointCloud("map",*scan_in,sensor_cloud,listener_);
		ROS_INFO("scan - sensor_msg %d datas", sensor_cloud.data.size());
		pub_scan_.publish(sensor_cloud);

		//sensor_msgs::PointCloud2 --> pcl::PointCloud
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(sensor_cloud,pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	        pcl::fromPCLPointCloud2(pcl_pc2,*final_cloud);
		ROS_INFO("pcl - pointcloud %d datas", final_cloud->points.size());
		pub_cloud_.publish(*final_cloud);

	       /*Creating the KdTree object for the search method of the extraction*/
               
               pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
               tree->setInputCloud (final_cloud);
               std::vector<pcl::PointIndices> cluster_indices;
               pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
               ec.setClusterTolerance (0.2); /*Set 20cm as a Maximum distance between points in a cluster*/
               ec.setMinClusterSize (3);
               ec.setMaxClusterSize (1000);
               ec.setSearchMethod (tree);
               ec.setInputCloud (final_cloud);
               ec.extract (cluster_indices);

	      double cluster_x_mean,cluster_y_mean,cluster_radius;
	      int marker_id = 1;

	      //marker
              visualization_msgs::MarkerArray marker_array; 
	      marker_array.markers.clear();

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
              {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
              for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (final_cloud->points[*pit]); 
                cloud_cluster->width  = cloud_cluster->points.size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                /*This cluster size*/
                cluster_size = cloud_cluster->points.size();

		cluster_x_mean = (cloud_cluster->points[0].x + cloud_cluster->points[cluster_size -1].x)/2;
                cluster_y_mean = (cloud_cluster->points[0].y + cloud_cluster->points[cluster_size -1].y)/2;
                cluster_radius = sqrt(pow( (cloud_cluster->points[0].x - cloud_cluster->points[cluster_size -1].x) ,2) + pow( (cloud_cluster->points[0].y - cloud_cluster->points[cluster_size -1].y),2))/2;
	       
                ROS_INFO("Cluster_info %d r:%f pos:(%f,%f)", cluster_size,cluster_radius,cluster_x_mean,cluster_y_mean); 
		visualization_msgs::Marker marker;
              	marker.type = visualization_msgs::Marker::CYLINDER;
		marker.id = marker_id;
              	marker.header.frame_id = "map";
              	marker.header.stamp = ros::Time();
              	marker.pose.orientation.x = 0.0;
              	marker.pose.orientation.y = 0.0;
              	marker.pose.orientation.z = 0.0;
              	marker.pose.orientation.w = 1.0;
              	marker.color.a = 1.0; // Don't forget to set the alpha!
              	marker.color.r = 0.0;
              	marker.color.g = 1.0;
              	marker.color.b = 0.0;

                marker.pose.position.x = cluster_x_mean;
		marker.pose.position.y = cluster_y_mean;
		marker.pose.position.z = 1;

                marker.scale.x = 2*cluster_radius ;
		marker.scale.y = 2*cluster_radius ;
		marker.scale.z = 0.1;

		if(std::fabs(cluster_x_mean) >= 1.75 || std::fabs(cluster_y_mean) >= 1.75 || cluster_radius >= 0.4){
                        marker.color.a = 0.3;
			marker.color.r = 0.9;
                	marker.color.g = 0.3;
                	marker.color.b = 0.3;
		}

		//if cluster is within the working space, publish it
		if(std::fabs(cluster_x_mean) < 1.75 || std::fabs(cluster_y_mean) < 1.75){
			pcl_obstacle_detection::ObjectPoint obj_point;
			obj_point.center.x = cluster_x_mean;
			obj_point.center.y = cluster_y_mean;
			obj_point.radius =  cluster_radius;
			pub_objPoint_.publish(obj_point);
		}


               //Push back marker
               marker_array.markers.push_back(marker);

	       marker_id ++;
               }
              ROS_INFO("Marker Array size: %d",marker_array.markers.size());	
              pub_markerArray_.publish(marker_array);

	}
};

int main(int argc, char** argv)
{
  ros::init (argc, argv, "cluster_node");
  ObstacleCluster app;
  app.run_it();
}

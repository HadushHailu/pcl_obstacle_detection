#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>


class ObstacleCluster{
    private:
	ros::NodeHandle nh_;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	ros::Publisher pub_scan_, pub_cloud_;
	ros::Subscriber sub_;
	laser_geometry::LaserProjection projector_;
    public:
	ObstacleCluster(){
		pub_cloud_ = nh_.advertise<PointCloud> ("/cluster_pointcloud2", 1);
		pub_scan_ = nh_.advertise<sensor_msgs::PointCloud2>("/cluster_scan",100);
		sub_ = nh_.subscribe("/scan", 100, &ObstacleCluster::scanCallback, this);
	}

	void run_it(){
		PointCloud::Ptr msg (new PointCloud);
	 	msg->header.frame_id = "some_tf_frame";
  		msg->height = msg->width = 1;
  		msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

 	 	ros::Rate loop_rate(4);
  		while (nh_.ok())
  		{
    			pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    			pub_cloud_.publish (msg);
    			ros::spinOnce ();
    			loop_rate.sleep ();
  		}

	}

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in){
		sensor_msgs::PointCloud2 sensor_cloud;
		//change LaserScan msg to PointCloud
		projector_.projectLaser(*scan_in, sensor_cloud);
		ROS_INFO("scan - sensor_msg %d datas", sensor_cloud.data.size());
		pub_scan_.publish(sensor_cloud);

		//sensor_msgs::PointCloud2 --> pcl::PointCloud
		pcl::PCLPointCloud2 pcl_pc2;
		pcl_conversions::toPCL(sensor_cloud,pcl_pc2);
		pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	        pcl::fromPCLPointCloud2(pcl_pc2,*final_cloud);
		ROS_INFO("pcl - pointcloud %d datas", final_cloud->points.size());
		pub_cloud_.publish(*final_cloud);
	
	}
};

int main(int argc, char** argv)
{
  ros::init (argc, argv, "cluster_node");
  ObstacleCluster app;
  app.run_it();
}

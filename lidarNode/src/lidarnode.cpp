#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using namespace std;

class LidarNode{
public:
	LidarNode();
	void Callback(const sensor_msgs::PointCloud2 &scan);

private:
	ros::NodeHandle node_handle;
	ros::Subscriber point_sub;
};

LidarNode::LidarNode(){
	ROS_INFO("initialize");
	point_sub = node_handle.subscribe("velodyne_points", 1028, &LidarNode::Callback, this);
}

void LidarNode::Callback(const sensor_msgs::PointCloud2 &scan){
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(scan,pcl_pc);
	pcl::PointCloud<pcl::PointXYZI>::Ptr teamp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromPCLPointCloud2(pcl_pc,*teamp);
	stringstream times;
	string rostime;
	times << ros::Time::now();
	times >> rostime;
	rostime = "/home/yxt/test/" + rostime;
	// write pointcloud data to file
	ofstream out;
	out.open(rostime.c_str(),ios::out|ios::app);

	for (int i = 0; i < teamp->size(); ++i) {
		out << teamp->points[i].x << " " << teamp->points[i].y << " " << teamp->points[i].z << " " << teamp->points[i].intensity << endl;
	}
	cout << "end." << endl;
}

int main(int argc, char **argv){

	srand(time(NULL));
	// ROS
	ros::init(argc,argv,"lidarnode");
	LidarNode node;
	ros::spin();

	return 0;
}
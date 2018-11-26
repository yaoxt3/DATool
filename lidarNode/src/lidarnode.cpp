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

string path = "/home/yxt/test/lidar/";

class LidarNode{
public:
	LidarNode();
	void Callback(const sensor_msgs::PointCloud2 &scan);

private:
	ros::NodeHandle node_handle;
	ros::Subscriber point_sub;
	int mcount;
};

LidarNode::LidarNode(){
	ROS_INFO("initialize");
	mcount = 0;
	point_sub = node_handle.subscribe("velodyne_points", 1028, &LidarNode::Callback, this);
}

void LidarNode::Callback(const sensor_msgs::PointCloud2 &scan){
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(scan,pcl_pc);
	pcl::PointCloud<pcl::PointXYZI>::Ptr teamp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromPCLPointCloud2(pcl_pc,*teamp);

	ostringstream filename;
	filename << path;
	filename << mcount << ".txt";
	// write pointcloud data to file
	ofstream out;
	out.open(filename.str().c_str(),ios::out|ios::app);

	for (int i = 0; i < teamp->size(); ++i) {
		out << teamp->points[i].x << " " << teamp->points[i].y << " " << teamp->points[i].z << " " << teamp->points[i].intensity << endl;
	}
	mcount++;
	cout << "file saved at " << filename.str() << endl;
}

int main(int argc, char **argv){

	srand(time(NULL));
	// ROS
	ros::init(argc,argv,"lidarnode");
	LidarNode node;
	ros::spin();

	return 0;
}
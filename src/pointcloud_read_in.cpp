
// Std Stuff
#include <iostream>
#include <fstream>
#include <istream>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <tiffio.h>

void normalize_pointcloud(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
	float min_x = pow(10,10);
	float min_y = pow(10,10);
	float min_z = pow(10,10);
	ROS_INFO_STREAM("size: " << cloud.size());
	for(int i=0; i<cloud.points.size(); i++)
	{
		if(cloud.points[i].x < min_x)
			min_x = cloud.points[i].x;
		if(cloud.points[i].y < min_y)
			min_y = cloud.points[i].y;
		if(cloud.points[i].z < min_z)
			min_z = cloud.points[i].z;
	}
	ROS_INFO_STREAM("min_x: " << min_x << "   min_y: " << min_y );
	for(int i=0; i<cloud.points.size(); i++)
	{
		cloud.points[i].x -= min_x;
		cloud.points[i].y -= min_y;
		cloud.points[i].z -= min_z;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "canopy_segmentation");
	ros::NodeHandle nh;

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   ros::console::notifyLoggerLevelsChanged();

	ros::Publisher cloud_pub;
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud_read_in", 1);

	std::string file_name;
	nh.param<std::string>("pointcloud_read_in/file_name", file_name, "/home/conor/Downloads/ECODSEdataset/ECODSEdataset/RSdata/pointCloud/ptcloud_OSBS_002.csv");
	std::ifstream file(file_name.c_str());

	std::string current_line;
	std::string x_val;
	std::string y_val;
	std::string z_val;

	getline(file, current_line); // remove first X Y Z values
	ROS_INFO_STREAM(current_line);

	sensor_msgs::PointCloud2 pointcloud_msg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pointcloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

	while ( file.good() )
	{
	     //getline(file, current_line); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
	     getline(file, x_val, ',');
	     getline(file, y_val, ',');
	     getline(file, z_val);

	     pcl::PointXYZ point;
	     point.x = strtof(x_val.c_str(),0);
	     point.y = strtof(y_val.c_str(),0); 
	     point.z = strtof(z_val.c_str(),0);

	     ROS_INFO_STREAM_THROTTLE(.01, point.x << ", \t" << point.y << ", \t" << point.z);

	     pcl_pointcloud_ptr->points.push_back(point);
	}

	normalize_pointcloud(*pcl_pointcloud_ptr);

	pcl::toROSMsg(*pcl_pointcloud_ptr, pointcloud_msg);
	pointcloud_msg.header.frame_id = "map";

	ROS_INFO_STREAM("published a cloud of size " << pointcloud_msg.height*pointcloud_msg.width);

	file.close();
	
	for(int i=0; i<30; i++)
		ROS_INFO_STREAM("cloud point " << i << " \tx: " << pcl_pointcloud_ptr->points[i].x << " \ty: " << pcl_pointcloud_ptr->points[i].y << " \tz: " << pcl_pointcloud_ptr->points[i].z);

	while(ros::ok())
	{
		cloud_pub.publish(pointcloud_msg);
		ros::Duration(0.5).sleep();
	}
}


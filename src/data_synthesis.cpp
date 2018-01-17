
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

//#include </usr/local/include/shapefil.h>
#include <shplib/shapefil.h>

sensor_msgs::PointCloud2 canopy_height_cloud_;
sensor_msgs::PointCloud2 rgb_cloud_;
sensor_msgs::PointCloud2 hyperspectral_cloud_;

void rgbCallback(const sensor_msgs::PointCloud2 pointcloud_in)
{
	rgb_cloud_ = pointcloud_in;
}

void canopyHeightCallback(const sensor_msgs::PointCloud2 pointcloud_in)
{
	canopy_height_cloud_ = pointcloud_in;
}

sensor_msgs::PointCloud2 buildCombinedCloud()
{
	ROS_INFO_STREAM("rgb size... " << rgb_cloud_.height*rgb_cloud_.width);
	ROS_INFO_STREAM("chm size... " << canopy_height_cloud_.height*canopy_height_cloud_.width);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	sensor_msgs::PointCloud2 output_cloud;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr canopy_height_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(rgb_cloud_, *rgb_cloud_ptr);
	pcl::fromROSMsg(canopy_height_cloud_, *canopy_height_cloud_ptr);

	float rgb_count = sqrt(rgb_cloud_.height*rgb_cloud_.width);
	float chm_count = sqrt(canopy_height_cloud_.height*canopy_height_cloud_.width);

	for(int i=0; i<chm_count; i++)
		for(int j=0; j<chm_count; j++)
		{
			ROS_INFO_STREAM(i << " " << j << " " << i*chm_count+j  << " " << i*rgb_count/chm_count*rgb_count+j*rgb_count/chm_count);
			pcl::PointXYZRGB point;
			point.x = canopy_height_cloud_ptr->points[i*chm_count+j].x;
			point.y = canopy_height_cloud_ptr->points[i*chm_count+j].y;
			point.z = canopy_height_cloud_ptr->points[i*chm_count+j].z;
			for(int k=0; k<rgb_count/chm_count; k++)
				for(int m=0; m<rgb_count/chm_count; m++)
				{
					pcl::PointXYZRGB offset_point = point;
					offset_point.x = point.x + (rgb_count/chm_count/2-k)/(rgb_count/chm_count);
					offset_point.y = point.y + (rgb_count/chm_count/2-m)/(rgb_count/chm_count);
					offset_point.r = rgb_cloud_ptr->points[i*rgb_count/chm_count*rgb_count+j*rgb_count/chm_count+rgb_count*m+k].r;
					offset_point.g = rgb_cloud_ptr->points[i*rgb_count/chm_count*rgb_count+j*rgb_count/chm_count+rgb_count*m+k].g;
					offset_point.b = rgb_cloud_ptr->points[i*rgb_count/chm_count*rgb_count+j*rgb_count/chm_count+rgb_count*m+k].b;
			
					output_cloud_ptr->points.push_back(offset_point);
				}
		}

	pcl::toROSMsg(*output_cloud_ptr, output_cloud);
	output_cloud.header.frame_id = "map";

	return output_cloud;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_synthesis");
	ros::NodeHandle nh;

	//SHPOpen("fish", "fish");

	ros::Subscriber canopy_height_sub = nh.subscribe<sensor_msgs::PointCloud2>("canopy_segmentation/canopy_height_cloud", 1, canopyHeightCallback);
	ros::Subscriber rgb_height_sub = nh.subscribe<sensor_msgs::PointCloud2>("canopy_segmentation/rgb_cloud", 1, rgbCallback);
	ros::Publisher combined_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("canopy_segmentation/combined_cloud", 1);
	sensor_msgs::PointCloud2 combined_cloud;
	

	bool cloud_made = false;
	while(ros::ok())
	{
		ros::spinOnce();
		if(canopy_height_cloud_.height*canopy_height_cloud_.width > 1 && rgb_cloud_.height*rgb_cloud_.width > 1) // && hyperspectral_cloud_.height*hyperspectral_cloud_.width > 1)
		{
			combined_cloud = buildCombinedCloud();
			cloud_made = true;
		}
		else
			ROS_WARN_STREAM("[CanopyDataSynthesis] Haven't yet gotten all the clouds...");
		ros::Duration(1).sleep();
		while(cloud_made && ros::ok())
		{
			ROS_INFO_STREAM("size... " << combined_cloud.height*combined_cloud.width);
			combined_cloud_pub.publish(combined_cloud);
			ros::Duration(1).sleep();
		}
	}


}
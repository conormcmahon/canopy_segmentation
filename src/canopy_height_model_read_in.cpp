
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>

#include "gdal/cpl_conv.h" // for CPLMalloc()

// Wall_Features
#include <wall_features/wall_damage_estimation.h>
#include <wall_features/wall_damage_estimation.hpp>
#include <wall_features/point_wall_damage.h>
#include <wall_features/wall_damage_histogram.h>

int main(int argc, char** argv)
{

	ros::init(argc, argv, "canopy_height_model_in");
	ros::NodeHandle nh;


    GDALDataset  *poDataset;
    GDALAllRegister();
    std::string dataset_number;
    nh.param<std::string>("canopy_segmentation/chm/dataset_number", dataset_number, "_002");
    std::string dataset_name = "/home/conor/Downloads/ECODSEdataset/ECODSEdataset/RSdata/chm/OSBS" + dataset_number + "_chm.tif";
    poDataset = (GDALDataset *) GDALOpen( dataset_name.c_str(), GA_ReadOnly );
    if( poDataset == NULL )
    {
        //...;
    }

    double        adfGeoTransform[6];
	printf( "Driver: %s/%s\n",
	        poDataset->GetDriver()->GetDescription(),
	        poDataset->GetDriver()->GetMetadataItem( GDAL_DMD_LONGNAME ) );
	printf( "Size is %dx%dx%d\n",
	        poDataset->GetRasterXSize(), poDataset->GetRasterYSize(),
	        poDataset->GetRasterCount() );
	if( poDataset->GetProjectionRef()  != NULL )
	    printf( "Projection is `%s'\n", poDataset->GetProjectionRef() );
	if( poDataset->GetGeoTransform( adfGeoTransform ) == CE_None )
	{
	    printf( "Origin = (%.6f,%.6f)\n",
	            adfGeoTransform[0], adfGeoTransform[3] );
	    printf( "Pixel Size = (%.6f,%.6f)\n",
	            adfGeoTransform[1], adfGeoTransform[5] );
	}


	printf("\n\n\n");

	GDALRasterBand  *poBand;
	int             nBlockXSize, nBlockYSize;
	int             bGotMin, bGotMax;
	double          adfMinMax[2];

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr raster_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int i=0; i<poDataset->GetRasterXSize(); i++)
		for (int j=0; j<poDataset->GetRasterYSize(); j++)
		{
			pcl::PointXYZRGB point;
			point.x = i;
			point.y = j;
			point.z = 0;
			raster_cloud_ptr->points.push_back(point);
		}

	for (int raster_band_id=1; raster_band_id<poDataset->GetRasterCount()+1; raster_band_id++)
	{
		poBand = poDataset->GetRasterBand( raster_band_id );
		poBand->GetBlockSize( &nBlockXSize, &nBlockYSize );
		printf( "Block=%dx%d Type=%s, ColorInterp=%s\n",
		        nBlockXSize, nBlockYSize,
		        GDALGetDataTypeName(poBand->GetRasterDataType()),
		        GDALGetColorInterpretationName(
		            poBand->GetColorInterpretation()) );
		adfMinMax[0] = poBand->GetMinimum( &bGotMin );
		adfMinMax[1] = poBand->GetMaximum( &bGotMax );
		if( ! (bGotMin && bGotMax) )
		    GDALComputeRasterMinMax((GDALRasterBandH)poBand, TRUE, adfMinMax);
		printf( "Min=%.3f, Max=%.3f\n", adfMinMax[0], adfMinMax[1] );
		if( poBand->GetOverviewCount() > 0 )
		    printf( "Band has %d overviews.\n", poBand->GetOverviewCount() );
		if( poBand->GetColorTable() != NULL )
		    printf( "Band has a color table with %d entries.\n",
		             poBand->GetColorTable()->GetColorEntryCount() );

		int   nXSize = poBand->GetXSize();
		int   nYSize = poBand->GetYSize();
		//pafScanline = (float *) CPLMalloc(sizeof(float)*nXSize); 		<-- probably a safer array initialization - figure out how to do this in 2D?  
		float pafScanline[nXSize][nYSize];
		CPLErr cplerr;
		cplerr = poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize,
	                  pafScanline, nXSize, nYSize, GDT_Float32,
	                  0, 0 );
		ROS_INFO_STREAM("raster band: " << raster_band_id << "  size: " << nXSize << "x" << nYSize);

		for (int i=0; i<nXSize; i++)
			for (int j=0; j<nYSize; j++)
			{
				switch(raster_band_id)
				{
					case 1:
						raster_cloud_ptr->points[i*nYSize+j].z = pafScanline[i][j];
						break;
				};
			}
	}

	sensor_msgs::PointCloud2 raster_cloud_msg;
	pcl::toROSMsg(*raster_cloud_ptr, raster_cloud_msg);
	raster_cloud_msg.header.frame_id = "map";

	ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("canopy_segmentation/canopy_height_cloud", 1);
/*	while(ros::ok())
	{
		cloud_pub.publish(raster_cloud_msg);
		ros::Duration(1).sleep();
	}
*/

// --------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------------------


  // --------------------------------------------- Initializing Things ---------------------------------------------

  // Publishers for Clouds
  ros::Publisher raster_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/wall_cloud", 1);
  ros::Publisher voxelized_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/voxelized_cloud", 1);
  ros::Publisher damage_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/damage_cloud", 1);
  ros::Publisher histogram_pub = nh.advertise<sensor_msgs::PointCloud2>("wall_features/histogram_cloud", 1);

  // ROS Msg Clouds
  sensor_msgs::PointCloud2 voxelized_msg;
  sensor_msgs::PointCloud2 damage_msg;
  sensor_msgs::PointCloud2 histogram_msg;

  // PCL Clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr canopy_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWallDamage>::Ptr wall_damage_cloud (new pcl::PointCloud<pcl::PointWallDamage>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelized_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::WallDamageHistogram>::Ptr histogram_cloud (new pcl::PointCloud<pcl::WallDamageHistogram>);

  // Feature Estimators 
  pcl::WallDamagePointwiseEstimation<pcl::PointXYZRGB, pcl::PointWallDamage> point_damage_estimator;
  pcl::WallDamageHistogramEstimation<pcl::PointWallDamage, pcl::PointXYZRGB, pcl::WallDamageHistogram> damage_histogram_estimator;

  // --------------------------------------------- Set 'Wall' Parameters ---------------------------------------------
  pcl::fromROSMsg(raster_cloud_msg, *canopy_cloud);

  // --------------------------------------------- Voxelization ---------------------------------------------
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  *voxelized_cloud = *canopy_cloud;
  vg.setInputCloud(voxelized_cloud);
  float leaf_size;
  nh.param<float>("wall_features/leaf_size", leaf_size, 0.5);
  ROS_ERROR_STREAM("leaf size: " << leaf_size);
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  pcl::PointCloud<pcl::PointXYZRGB> temp_pc;
  vg.filter(temp_pc);
  *voxelized_cloud = temp_pc; 
  ROS_INFO_STREAM("[WallFeatures] Performed voxelization of input cloud - output cloud size is " << voxelized_cloud->points.size());

  // --------------------------------------------- Clipping ---------------------------------------------
  float clipping_height;
  nh.param<float>("canopy_segmentation/chm/clipping_height", clipping_height, 10);
  pcl::CropBox<pcl::PointXYZRGB> crop;
  crop.setInputCloud(voxelized_cloud);
  // Set dimensions of clipping box:
  Eigen::Vector4f min_point = Eigen::Vector4f(-100, -100, clipping_height, 0);
  Eigen::Vector4f max_point = Eigen::Vector4f(100, 100, 100, 0);
  crop.setMin(min_point);
  crop.setMax(max_point);
  // Set pose of clipping box: 
  Eigen::Vector3f translation = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f rotation = Eigen::Vector3f(0, 0, 0);
  crop.setTranslation(translation);
  crop.setRotation(rotation);   
  crop.setKeepOrganized(false);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_pcp(new pcl::PointCloud<pcl::PointXYZRGB>());
  crop.filter(*temp_pcp);
  *voxelized_cloud = *temp_pcp;

  // --------------------------------------------- Wall Damage Estimation ---------------------------------------------
  float wall_coeffs[4];
  wall_coeffs[0] = 0;
  wall_coeffs[1] = 0;
  wall_coeffs[2] = 1;
  wall_coeffs[3] = 0;
  point_damage_estimator.setWallCoefficients(wall_coeffs);
  int k_search;
  nh.param<int>("wall_features/k_search_histogram", k_search, 30);
  point_damage_estimator.setKSearch(k_search);
  point_damage_estimator.compute(*voxelized_cloud, *wall_damage_cloud);
  ROS_INFO_STREAM("[WallFeatures] Performed pointwise damage estimation - output cloud size is " << wall_damage_cloud->points.size());

  // --------------------------------------------- Histogram Estimation ---------------------------------------------
  float lower_angle_bin_limit, upper_angle_bin_limit, lower_dist_bin_limit, upper_dist_bin_limit;
  bool automatically_set_bins;
  nh.param<bool>("wall_features/automatically_set_bins", automatically_set_bins, true);
  if(automatically_set_bins)
  {
    lower_angle_bin_limit = wall_damage_cloud->points[0].angle_offset;
    upper_angle_bin_limit = lower_angle_bin_limit;
    lower_dist_bin_limit = wall_damage_cloud->points[0].dist_offset;
    upper_dist_bin_limit = lower_dist_bin_limit;
    for(int i=0; i<wall_damage_cloud->size(); i++)
    {
      if(wall_damage_cloud->points[i].angle_offset < lower_angle_bin_limit)
        lower_angle_bin_limit = wall_damage_cloud->points[i].angle_offset;
      if(wall_damage_cloud->points[i].angle_offset > upper_angle_bin_limit)
        upper_angle_bin_limit = wall_damage_cloud->points[i].angle_offset;
      if(wall_damage_cloud->points[i].dist_offset < lower_dist_bin_limit)
        lower_dist_bin_limit = wall_damage_cloud->points[i].dist_offset;
      if(wall_damage_cloud->points[i].dist_offset > upper_dist_bin_limit)
        upper_dist_bin_limit = wall_damage_cloud->points[i].dist_offset;
    }
  }
  else
  {
    nh.param<float>("wall_features/lower_angle_bin_limit", lower_angle_bin_limit, 0);
    nh.param<float>("wall_features/upper_angle_bin_limit", upper_angle_bin_limit, 3.14159);
    nh.param<float>("wall_features/lower_dist_bin_limit", lower_dist_bin_limit, -0.02);
    nh.param<float>("wall_features/upper_dist_bin_limit", upper_dist_bin_limit, 0.02);
  }
  ROS_ERROR_STREAM(lower_angle_bin_limit << " " << upper_angle_bin_limit << " " << lower_dist_bin_limit << " " << upper_dist_bin_limit);
  damage_histogram_estimator.setBinLimits(lower_angle_bin_limit, upper_angle_bin_limit, lower_dist_bin_limit, upper_dist_bin_limit);
  nh.param<int>("wall_features/k_search_histogram", k_search, 30);
  damage_histogram_estimator.setKSearch(k_search);
  damage_histogram_estimator.compute(*wall_damage_cloud, *voxelized_cloud, *histogram_cloud);
  ROS_INFO_STREAM("[WallFeatures] Performed histogram cloud estimation.");

  pcl::toROSMsg(*voxelized_cloud, voxelized_msg);
  pcl::toROSMsg(*wall_damage_cloud, damage_msg);
  pcl::toROSMsg(*histogram_cloud, histogram_msg);

  voxelized_msg.header.frame_id = "map";
  damage_msg.header.frame_id = "map";
  histogram_msg.header.frame_id = "map";

  while(ros::ok())
  {
    cloud_pub.publish(raster_cloud_msg);
    voxelized_pub.publish(voxelized_msg);
    damage_pub.publish(damage_msg);
    histogram_pub.publish(histogram_msg);
    ros::Duration(1.0).sleep();
  }




}
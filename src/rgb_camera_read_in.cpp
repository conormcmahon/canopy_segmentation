
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <gdal/gdal.h>
#include <gdal/gdal_priv.h>

#include "gdal/cpl_conv.h" // for CPLMalloc()

int main(int argc, char** argv)
{

	ros::init(argc, argv, "rgb_camera_in");
	ros::NodeHandle nh;


    GDALDataset  *poDataset;
    GDALAllRegister();
    std::string dataset_number;
    nh.param<std::string>("canopy_segmentation/rgb/dataset_number", dataset_number, "_002");
    std::string dataset_name = "/home/conor/Downloads/ECODSEdataset/ECODSEdataset/RSdata/camera/OSBS" + dataset_number + "_camera.tif";
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
					raster_cloud_ptr->points[i*nYSize+j].r = pafScanline[i][j];
					break;
				case 2:
					raster_cloud_ptr->points[i*nXSize+j].g = pafScanline[i][j];
					break;
				case 3:
					raster_cloud_ptr->points[i*nXSize+j].b = pafScanline[i][j];
					break; 
			};
		}
}

for (int i=0; i<poDataset->GetRasterXSize(); i++)
	for (int j=0; j<poDataset->GetRasterYSize(); j++)
	{
		raster_cloud_ptr->points[i*poDataset->GetRasterXSize()+j].x = raster_cloud_ptr->points[i*poDataset->GetRasterXSize()+j].x/4;
		raster_cloud_ptr->points[i*poDataset->GetRasterYSize()+j].y = raster_cloud_ptr->points[i*poDataset->GetRasterYSize()+j].y/4;
	}

sensor_msgs::PointCloud2 raster_cloud_msg;
pcl::toROSMsg(*raster_cloud_ptr, raster_cloud_msg);
raster_cloud_msg.header.frame_id = "map";

ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("canopy_segmentation/rgb_cloud", 1);
while(ros::ok())
{
	cloud_pub.publish(raster_cloud_msg);
	ros::Duration(1).sleep();
}


}
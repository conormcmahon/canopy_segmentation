
// ROS Stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// Pointcloud Library
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// PNG Library
#include <IL/il.h>
#include <IL/ilu.h>
#include <IL/ilut.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "data_synthesis");
	ros::NodeHandle nh;

	//ilInit();
/*	printf("DevIL has been initialized\n");

	// Loading an image
	ILboolean result = ilLoadImage( "4px.png" ) ;

	if( result == true )
	{
		printf("the image loaded successfully\n");
	}
	else
	{
		printf("The image failed to load\n" ) ;

		ILenum err = ilGetError() ;
		printf( "the error %d\n", err );
		printf( "string is %s\n", ilGetString( err ) );
	}

	int size = ilGetInteger( IL_IMAGE_SIZE_OF_DATA ) ;
	printf("Data size:  %d\n", size );
	ILubyte * bytes = ilGetData() ;

	for( int i = 0 ; i < size; i++ )
	{
		// see we should see the byte data of the image now.
		printf( "%d\n", bytes[ i ] );
	}*/

}
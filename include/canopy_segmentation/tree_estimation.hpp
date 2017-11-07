

#ifndef IMPL_TREE_ESTIMATION_
#define IMPL_TREE_ESTIMATION_

#include "tree_estimation.h"

/* temp - list of parameters to be determined:
  -- PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  -- PCL_ADD_NORMAL4D; 				// This adds the member normal[3] which can also be accessed using the point (which is float[4])
  float angle_offset_avg; 			// Average offset in angle between local normal vectors of each neighbor point and the expected normal vector of the containing plane primitive definition
  float dist_offset_avg; 			// Average offset in position between each neighbor point and the containing plane primitive definition
  float	histogram[80];
*/
namespace CanopyInspection
{ 
// -----------------------------------------------------------------------------------------------------------------------------
// ------------------------------------------------ Pointwise Damage Estimation ------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
	template <typename PointInT, typename PointOutT> void TreeEstimation <PointInT, PointOutT>::computeFeature (pcl::PointCloud<PointOutT> &output) { }
	//template <typename PointInT> void setInputCloud(const pcl::PointCloud<PointInT> &input);

	template <typename PointInT, typename PointOutT> void 
	TreeEstimation<PointInT, PointOutT>::compute (const pcl::PointCloud<PointInT> &input, 
																pcl::PointCloud<PointOutT> &output)
	{ 
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();  

		
	}

}

#endif // IMPL_TREE_ESTIMATION_
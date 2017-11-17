

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
		int num_bands = 426;

		// Low NDVI values occur in unvegetated areas. NDVI ranges from -1 to 1 .
		//   Unvegetated areas within a 'tree' may be gaps in the canopy erroneously included. 
		//   Exclude pixels with NDVI values below the following:
		float min_NDVI = 0.3; 
		int red_band = 55;
		int nir_band = 90;

		float min_height = 0;// 1;

		output.points.clear();

		CanopyInspection::TreeSpectrumAverage spectrum_average_pixel;

		spectrum_average_pixel.avg_height = 0;
		spectrum_average_pixel.max_height = 0;
		spectrum_average_pixel.min_height = 100;

		for(int band_index=0; band_index<num_bands; band_index++)
			spectrum_average_pixel.hyperspectral_data[band_index] = 0;
		for(int pixel_index=0; pixel_index<input.points.size(); pixel_index++)
		{
			float red_band_val = input.points[pixel_index].hyperspectral_data[red_band];
			float nir_band_val = input.points[pixel_index].hyperspectral_data[nir_band];
			float NDVI = (nir_band_val-red_band_val)/(nir_band_val+red_band_val);
			if( NDVI > min_NDVI )// && input.points[pixel_index].z > min_height)
			{
				for(int band_index=0; band_index<num_bands; band_index++)
				{
					if( isnan(input.points[pixel_index].hyperspectral_data[band_index]) )
						ROS_ERROR_STREAM("[TreeEstimation] In the input tree, band " << band_index << " of pixel " << pixel_index << " is nan." );
					else
						spectrum_average_pixel.hyperspectral_data[band_index] += input.points[pixel_index].hyperspectral_data[band_index];
				}
				spectrum_average_pixel.avg_height += input.points[pixel_index].z;
				if(input.points[pixel_index].z > spectrum_average_pixel.max_height)
					spectrum_average_pixel.max_height = input.points[pixel_index].z;
				if(input.points[pixel_index].z < spectrum_average_pixel.min_height)
					spectrum_average_pixel.min_height = input.points[pixel_index].z;
			}
		}
		for(int band_index=0; band_index<num_bands; band_index++)
			spectrum_average_pixel.hyperspectral_data[band_index] /= input.points.size();
		spectrum_average_pixel.avg_height /= input.points.size();

		output.points.push_back(spectrum_average_pixel);
	}

}

#endif // IMPL_TREE_ESTIMATION_
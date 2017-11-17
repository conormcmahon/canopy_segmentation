


#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include "canopy_segmentation/tree_spectrum_average.h"
#include "canopy_segmentation/canopy_inspection_pixel.h"
#include "canopy_segmentation/tree_estimation.h"
#include "canopy_segmentation/tree_estimation.hpp"

template class CanopyInspection::TreeEstimation<CanopyInspection::CanopyInspectionPixel, CanopyInspection::TreeSpectrumAverage>;
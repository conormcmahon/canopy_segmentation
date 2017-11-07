

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Choosing to encode calculated angle_offset for each point, and retain normal information, even though these are functionally redundant for us
// Choosing NOT to encode gross plane information in each point since it's the same across all of them - just need to make sure to manage this intelligently
//   Note that the angle and dist offsets are only meaningful given a fixed external reference plane! 

namespace CanopyInspection {
struct TreeSpectrumAverage
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float min_radius;
  float max_radius;
  float avg_height;
  float spectrum_average[426];      // Average Value in each Band (wavelength) from hyperspectral pixels in the tree

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}
POINT_CLOUD_REGISTER_POINT_STRUCT (CanopyInspection::TreeSpectrumAverage, 
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, min_radius, min_radius)
                                   (float, max_radius, max_radius)
                                   (float, avg_height, avg_height)
                                   (float[426], spectrum_average, spectrum_average)
)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const CanopyInspection::TreeSpectrumAverage& p);
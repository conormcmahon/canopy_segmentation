

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

// Choosing to encode calculated angle_offset for each point, and retain normal information, even though these are functionally redundant for us
// Choosing NOT to encode gross plane information in each point since it's the same across all of them - just need to make sure to manage this intelligently
//   Note that the angle and dist offsets are only meaningful given a fixed external reference plane! 

namespace CanopyInspection {
struct CanopyInspectionPixel
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  int crown_id;
  float RGB[3];						// RGB camera data 
  float hyperspectral_data[426];    // Hyperspectral camera data

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

}
POINT_CLOUD_REGISTER_POINT_STRUCT (CanopyInspection::CanopyInspectionPixel, 
                                   (float, crown_id, crown_id)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[3], RGB, RGB)
                                   (float[426], hyperspectral_data, hyperspectral_data)
)
PCL_EXPORTS std::ostream& operator << (std::ostream& os, const CanopyInspection::CanopyInspectionPixel& p);
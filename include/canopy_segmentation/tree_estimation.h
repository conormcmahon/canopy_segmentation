

#ifndef TREE_ESTIMATION_
#define TREE_ESTIMATION_

#include <pcl/features/feature.h>
#include <pcl/common/centroid.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

typedef pcl::PointXYZRGB PCLPoint;
typedef pcl::PointCloud<pcl::PointNormal> PCN;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr PCNP;

namespace CanopyInspection
{

	template <typename PointInT, typename PointOutT>
	class TreeEstimation//: public Feature<PointInT, PointOutT>
	{
	public:
		typedef boost::shared_ptr<TreeEstimation<PointInT,PointOutT> > Ptr;
		typedef boost::shared_ptr<const TreeEstimation<PointInT,PointOutT> > ConstPtr;
		/*using Feature<PointInT, PointOutT>::feature_name_;
        using Feature<PointInT, PointOutT>::getClassName;
        using Feature<PointInT, PointOutT>::indices_;
        using Feature<PointInT, PointOutT>::input_;
        using Feature<PointInT, PointOutT>::surface_;
        using Feature<PointInT, PointOutT>::k_;
        using Feature<PointInT, PointOutT>::search_radius_;
        using Feature<PointInT, PointOutT>::search_parameter_; */
        //typedef typename Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
        //typedef typename Feature<PointInT, PointOutT>::PointCloudConstPtr PointCloudConstPtr;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointCloudPtr;

		/** \brief Empty constructor. */
		TreeEstimation () 
		{
			//feature_name_ = "WallDamageEstimation";
		};

		/** \brief Empty destructor */
		virtual ~TreeEstimation () {}
		void compute (const pcl::PointCloud<PointInT> &input, pcl::PointCloud<PointOutT> &output);
		void computeFeature (pcl::PointCloud<PointOutT> &output);
		
		virtual inline void 
        setInputCloud (const PointCloudPtr &cloud)
        { /*
          input_ = cloud;
          if (use_sensor_origin_)
          {
            vpx_ = input_->sensor_origin_.coeff (0);
            vpy_ = input_->sensor_origin_.coeff (1);
            vpz_ = input_->sensor_origin_.coeff (2);
          } */
        }
	};

};

#ifdef PCL_NO_PRECOMPILE
#include "wall_features/wall_damage_estimation.hpp"
#endif // PCL_NO_PRECOMPILE

#endif // TREE_ESTIMATION_

// Std Stuff
#include <iostream>
#include <fstream>
#include <istream>

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include "canopy_segmentation/tree_spectrum_average.h"
#include "canopy_segmentation/canopy_inspection_pixel.h"
#include "canopy_segmentation/tree_estimation.h"

// Produces estimates of the likelihood of an input TreeSpectrumAverage pointcloud being any one of the mapped input species
std::map< std::string, std::vector<float> > tree_avg_spectral_angle_classifier(	std::vector<std::string> species_keys, 
																				std::map< std::string, pcl::PointCloud<CanopyInspection::TreeSpectrumAverage> > tree_spectrum_map, 
																				CanopyInspection::TreeSpectrumAverage input_tree_spectrum_average, 
																				std::vector<int> target_bands )
{
	std::map< std::string, std::vector<float> > tree_spectral_angles;
	ROS_INFO_STREAM("received classifier call");
	float input_spectrum_mag = 0;
	for(int band=0; band<target_bands.size(); band++)
	{
		if( isnan( pow(input_tree_spectrum_average.hyperspectral_data[target_bands[band]],2) ) )
			ROS_ERROR_STREAM("input cloud nan at band " << target_bands[band] << " " << input_tree_spectrum_average.hyperspectral_data[target_bands[band]]);
		else
			input_spectrum_mag += pow(input_tree_spectrum_average.hyperspectral_data[target_bands[band]],2);
		ROS_ERROR_STREAM("band: " << target_bands[band] << " value: " << input_tree_spectrum_average.hyperspectral_data[target_bands[band]] << " squared: " << pow(input_tree_spectrum_average.hyperspectral_data[target_bands[band]],2) << " sum: " << input_spectrum_mag);
	}
	input_spectrum_mag = sqrt(input_spectrum_mag);

	std::map< std::string, float > species_mean_angle;
	std::map< std::string, float > species_min_angle;
	std::map< std::string, float > species_avg_height;
	std::map< std::string, float > species_max_height;
	std::map< std::string, float > species_min_height;

	for(int species_index=0; species_index<species_keys.size(); species_index++)
	{
		species_mean_angle[species_keys[species_index]] = 0;
		species_min_angle[species_keys[species_index]] = 100;
		species_avg_height[species_keys[species_index]] = 0;
		species_max_height[species_keys[species_index]] = 0;
		species_min_height[species_keys[species_index]] = 100;

		for(int tree_index=0; tree_index<tree_spectrum_map[species_keys[species_index]].size(); tree_index++)
		{
			tree_spectral_angles[species_keys[species_index]].push_back(0);
			float target_spectrum_mag = 0;
			//ROS_INFO_STREAM("tree stuffs" << tree_spectral_angles[species_keys[species_index]].size() << " " << tree_spectral_angles[species_keys[species_index]][tree_index] << " " << tree_index);
			for(int band=0; band<target_bands.size(); band++)
			{
				// Square of the difference between the expected and input band value
				//ROS_INFO_STREAM("floop");
				//ROS_INFO_STREAM(pow( tree_spectrum_map[species_keys[species_index]].points[tree_index].hyperspectral_data[target_bands[band]] - input_tree_spectrum_average.hyperspectral_data[target_bands[band]], 2));
				//ROS_INFO_STREAM(tree_spectrum_map[species_keys[species_index]].points[tree_index].hyperspectral_data[target_bands[band]]);
				//ROS_INFO_STREAM(input_tree_spectrum_average.hyperspectral_data[target_bands[band]]);
				if(isnan( tree_spectrum_map[species_keys[species_index]].points[tree_index].hyperspectral_data[target_bands[band]] - input_tree_spectrum_average.hyperspectral_data[target_bands[band]] ))
					ROS_ERROR_STREAM("species: " << species_keys[species_index] << " tree_ind " << tree_index << " band " << target_bands[band] );
				else
				{
					tree_spectral_angles[species_keys[species_index]][tree_index] += (pow( tree_spectrum_map[species_keys[species_index]].points[tree_index].hyperspectral_data[target_bands[band]] - input_tree_spectrum_average.hyperspectral_data[target_bands[band]], 2));
					target_spectrum_mag += pow(tree_spectrum_map[species_keys[species_index]].points[tree_index].hyperspectral_data[target_bands[band]], 2);
				}
			}
			target_spectrum_mag = sqrt(target_spectrum_mag);
			tree_spectral_angles[species_keys[species_index]][tree_index] /= (target_spectrum_mag*input_spectrum_mag);
			//ROS_INFO_STREAM(tree_index << " " << species_keys[species_index] << " " << tree_spectral_angles[species_keys[species_index]][tree_index]);
			species_mean_angle[species_keys[species_index]] += tree_spectral_angles[species_keys[species_index]][tree_index];
			if( species_min_angle[species_keys[species_index]] > tree_spectral_angles[species_keys[species_index]][tree_index] && tree_spectral_angles[species_keys[species_index]][tree_index] != 0 )
				species_min_angle[species_keys[species_index]] = tree_spectral_angles[species_keys[species_index]][tree_index];
			
			species_avg_height[species_keys[species_index]] += tree_spectrum_map[species_keys[species_index]].points[tree_index].max_height;
			if(species_max_height[species_keys[species_index]] < tree_spectrum_map[species_keys[species_index]].points[tree_index].max_height)
				species_max_height[species_keys[species_index]] = tree_spectrum_map[species_keys[species_index]].points[tree_index].max_height;
			if(species_min_height[species_keys[species_index]] > tree_spectrum_map[species_keys[species_index]].points[tree_index].max_height)
				species_min_height[species_keys[species_index]] = tree_spectrum_map[species_keys[species_index]].points[tree_index].max_height;
		}
		species_mean_angle[species_keys[species_index]] /= tree_spectrum_map[species_keys[species_index]].size();
		species_avg_height[species_keys[species_index]] /= tree_spectrum_map[species_keys[species_index]].size();
	}

	for(int species_index=0; species_index<species_keys.size(); species_index++)
	{
		ROS_INFO_STREAM(species_keys[species_index] << ": mean " << species_mean_angle[species_keys[species_index]] << ", min " << species_min_angle[species_keys[species_index]] );
		ROS_INFO_STREAM("   " << species_avg_height[species_keys[species_index]] << " " << species_max_height[species_keys[species_index]] << " " << species_min_height[species_keys[species_index]]);
	}

	return tree_spectral_angles;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "canopy_segmentation");
	ros::NodeHandle nh;

//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
 //   ros::console::notifyLoggerLevelsChanged();


	// --------------------------- Read-in of Crown ID Species Data ---------------------------
	std::string file_name;
	nh.param<std::string>("pointcloud_read_in/trees_file_name", file_name, "/home/conor/Downloads/ECODSEdataset/ECODSEdataset/Task3/GroundData/species_id_train.csv");
	std::ifstream file(file_name.c_str());

	std::string current_line;
	int crown_id;
	std::string species_id;
	std::string placeholder;
	std::map<int, std::string> crown_id_map;
	std::vector<std::string> species_id_list;
	std::vector<int> crown_id_list;

	getline(file, current_line); // remove first row - just contains column headers 
	ROS_INFO_STREAM(current_line); // display that row

	while ( file.good() && ros::ok() )
	{
	     //getline(file, current_line); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
	     getline(file, placeholder, ',');
	     crown_id = strtof(placeholder.c_str(),0);
	     getline(file, placeholder, ',');	// binomial name
	     getline(file, placeholder, ',');	// genus
	     getline(file, species_id, ',');
	     getline(file, placeholder);

	     ROS_INFO_STREAM(crown_id << " " << " " << species_id);

	     crown_id_map[crown_id] = species_id;
	     
	     bool id_used_already = false;
	     for(int i=0; i<species_id_list.size(); i++)
	     	if(species_id_list[i] == species_id)
	     	{
	     		id_used_already = true;
	     		break;
	     	}
     	if(!id_used_already)
     		species_id_list.push_back(species_id);
     	crown_id_list.push_back(crown_id);

	}

	ROS_INFO_STREAM("\n\n\n");

	file.close();
	
	// --------------------------- Read-in of Crown ID Species Data ---------------------------
	nh.param<std::string>("pointcloud_read_in/data_file_name", file_name, "/home/conor/Downloads/ECODSEdataset/ECODSEdataset/Task3/GroundData/hyper_bands_train.csv");
	std::ifstream data_file(file_name.c_str());

	getline(data_file, current_line); // remove first row - just contains column headers 
	ROS_INFO_STREAM(current_line); // display that row

	std::string height;
	std::string band_value;
	// All the CanopyInspectionPixels in one cloud - X position is index in a tree, Y position is tree index
	pcl::PointCloud<CanopyInspection::CanopyInspectionPixel> pixel_cloud;
	// Map to Average Spectra for each individual tree (keyed by species) 
	pcl::PointCloud<CanopyInspection::TreeSpectrumAverage> tree_average_cloud;
	// Map to Vectors of CanopyInspectionPixel clouds (keyed by species)
	std::map< std::string, std::vector< pcl::PointCloud<CanopyInspection::CanopyInspectionPixel> > > tree_pixel_clouds;
	// Map to TreeSpectrumAverage clouds (keyed by species)
	std::map< std::string, pcl::PointCloud<CanopyInspection::TreeSpectrumAverage> > species_tree_averages;
	// Estimator to average Spectral Bands across a Tree
	CanopyInspection::TreeEstimation<CanopyInspection::CanopyInspectionPixel, CanopyInspection::TreeSpectrumAverage> tree_estimator;
	float previous_crown_id = 0;
	while ( data_file.good() && ros::ok() )
	{
		//getline(file, current_line); // read a string until next comma: http://www.cplusplus.com/reference/string/getline/
		getline(data_file, placeholder, ',');
		crown_id = strtof(placeholder.c_str(),0);
		if(crown_id != previous_crown_id)
		{
			if(tree_pixel_clouds[crown_id_map[crown_id]].size() > 0)
			{
				pcl::PointCloud<CanopyInspection::TreeSpectrumAverage> new_tree;
				tree_estimator.compute((tree_pixel_clouds[crown_id_map[crown_id]][tree_pixel_clouds[crown_id_map[crown_id]].size()-1]),  new_tree);

				new_tree.points[0].x = tree_pixel_clouds[crown_id_map[crown_id]].size();
				new_tree.points[0].y = 0;
				new_tree.points[0].z = 0;//new_tree.points[0].avg_height;
				tree_average_cloud.push_back(new_tree.points[0]);
				species_tree_averages[crown_id_map[crown_id]].push_back(new_tree.points[0]);
			}
			pcl::PointCloud<CanopyInspection::CanopyInspectionPixel> temp_cloud;
			tree_pixel_clouds[crown_id_map[crown_id]].push_back( temp_cloud );
			previous_crown_id = crown_id;
		}
		CanopyInspection::CanopyInspectionPixel current_pixel;
		getline(data_file, height, ',');
		current_pixel.x = tree_pixel_clouds[crown_id_map[crown_id]][tree_pixel_clouds[crown_id_map[crown_id]].size()-1].points.size();
		current_pixel.y = tree_pixel_clouds[crown_id_map[crown_id]].size();
		current_pixel.z = strtof(height.c_str(),0);
		for(int i=0; i<426; i++)
		{
			getline(data_file, band_value, ',');
			//ROS_INFO_STREAM(i << " " << band_value);
			if(band_value == "NA")
				current_pixel.hyperspectral_data[i] = 0;
			else
				current_pixel.hyperspectral_data[i] = strtof(band_value.c_str(),0); 
		}
		pixel_cloud.points.push_back(current_pixel);

		current_pixel.y = 0;
		tree_pixel_clouds[crown_id_map[crown_id]][tree_pixel_clouds[crown_id_map[crown_id]].size()-1].points.push_back(current_pixel);		

		getline(data_file, placeholder);

		//ROS_INFO_STREAM(crown_id << tree_pixel_clouds[crown_id_map[crown_id]][tree_pixel_clouds[crown_id_map[crown_id]].size()-1].size() << ": " << crown_id_map[crown_id] << " " << height << " " << " " << band_value);
		

	}
	ROS_INFO_STREAM(pixel_cloud.points.size());

	ROS_INFO_STREAM(tree_average_cloud.points.size() << " " << pixel_cloud.points.size() << " " << tree_pixel_clouds.size());

	std::string target_species = "PIPA";
	float tree_index = 0;
	int min_band, max_band;
	nh.getParam("canopy_segmentation/tree_classifier/target_species", target_species);
	nh.getParam("canopy_segmentation/tree_classifier/tree_index", tree_index);
	nh.param<int>("canopy_segmentation/tree_classifier/min_band", min_band, 0);
	nh.param<int>("canopy_segmentation/tree_classifier/max_band", max_band, 75);

	std::vector<int> target_bands;
	for(int i=min_band; i<max_band; i++)
		target_bands.push_back(i);
	
	tree_avg_spectral_angle_classifier(species_id_list, species_tree_averages, species_tree_averages[target_species].points[tree_index], target_bands);

	while(ros::ok())
	{
		ros::Duration(0.5).sleep();
	}
}


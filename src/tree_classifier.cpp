
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

std::vector< std::vector<float> > scalar_multiplication(std::vector< std::vector<float> > input, float scalar)
{
	std::vector< std::vector<float> > output(input.size());
	for(int i=0; i<input.size(); i++)
		for(int j=0; j<input[0].size(); j++)
			output[i].push_back(input[i][j]*scalar);
	return output;
}

// Matrix multiplication using std::vector; assumes first dimension of vectors is 'columns'
std::vector< std::vector<float> > matrix_multiplication(std::vector< std::vector<float> > vec_1, std::vector< std::vector<float> > vec_2)
{
	std::vector< std::vector<float> > output(vec_1.size());
	for(int i=0; i<output.size(); i++)
		output[i] = std::vector<float>(vec_2[0].size());

	for(int i=0; i<output.size(); i++)
		for(int j=0; j<output[0].size(); j++)
		{
			output[i][j] = 0;
			for(int k=0; k<vec_2.size(); k++)
			{
				output[i][j] += vec_1[i][k]*vec_2[k][j];
			}
		}

	return output;
}

std::vector< std::vector<float> > matrix_subtraction(std::vector< std::vector<float> > vec_1, std::vector< std::vector<float> > vec_2)
{
	std::vector< std::vector<float> > output(vec_1.size());
	for(int i=0; i<vec_1.size(); i++)
		for(int j=0; j<vec_1[0].size(); j++)
			output[i].push_back(vec_1[i][j] - vec_2[i][j]);
	return output;
}

std::vector< std::vector<float> > matrix_exponent(std::vector< std::vector<float> > input, float base)
{
	std::vector< std::vector<float> > output(input.size());
	for(int i=0; i<input.size(); i++)
		for(int j=0; j<input[0].size(); j++)
			output[i].push_back(pow(base, input[i][j]));
	return output;
}

std::vector< std::vector<float> > matrix_exponent_2(std::vector< std::vector<float> > input, float exponent)
{
	std::vector< std::vector<float> > output(input.size());
	for(int i=0; i<input.size(); i++)
		for(int j=0; j<input[0].size(); j++)
			output[i].push_back(pow(input[i][j],exponent));
	return output;
}

std::vector< std::vector<float> > column_matrix(std::vector<float> input)
{
	std::vector< std::vector<float> > output(input.size());
	for(int i=0; i<input.size(); i++)
		output[i].push_back(input[i]);
	return output;
}

std::vector< std::vector<float> > matrix_transpose(std::vector< std::vector<float> > input)
{
	std::vector< std::vector<float> > output(input[0].size());
	for(int i=0; i<input[0].size(); i++)
		for(int j=0; j<input.size(); j++)
		{
			output[i].push_back(input[j][i]);
		}
	return output;
}

float matrix_determinant(std::vector< std::vector<float> > input)
{
	if(input.size() != input[0].size())
	{
		ROS_ERROR_STREAM("Error - matrix entered for determinant function not square.");
		return -1;
	}

	float output = 0;
	if(input.size()==2)
		output = input[0][0]*input[1][1] - input[0][1]*input[1][0];
	else 
		for(int i=0; i<input.size(); i++)
		{
			std::vector< std::vector<float> > sub_matrix(input.size()-1);
			for(int j=0; j<sub_matrix.size(); j++)
				for(int k=0; k<input.size(); k++)
					if(k!=i)
						sub_matrix[j].push_back(input[j+1][k]);
			output += pow(-1,i+1)*matrix_determinant(sub_matrix)*input[0][i];
		}
	return output;
}

void print_matrix(std::vector< std::vector<float> > input)
{
	std::cout << "Printing " << input.size() << " x " << input[0].size() << " matrix.\n";
	for(int i=0; i<input.size(); i++)
	{
		for(int j=0; j<input[0].size(); j++)
			std::cout << " \t" << input[i][j];
		std::cout << "\n";
	}
}


/*
   Find the cofactor matrix of a square matrix
   Modified from https://www.cs.rochester.edu/~brown/Crypto/assts/projects/adj.html
*/
std::vector< std::vector<float> > matrix_cofactor(std::vector< std::vector<float> > input)
{

   	std::vector< std::vector<float> > output(input.size());
	for (int i=0; i<input.size(); i++)
		output[i] = std::vector<float>(input.size());
   	
   	std::vector< std::vector<float> > temp(input.size()-1);
	for (int i=0; i<input.size()-1; i++)
		temp[i] = std::vector<float>(input.size()-1);

	for (int j=0; j<input.size(); j++) {
		for (int i=0; i<input.size(); i++) {
			/* Form the adjoint a_ij */
			int i1 = 0;
			for (int ii=0; ii<input.size(); ii++) {
				if (ii == i)
					continue;
				int j1 = 0;
				for (int jj=0; jj<input.size(); jj++) {
					if (jj == j)
						continue;
					temp[i1][j1] = input[ii][jj];
					j1++;
				}
				i1++;
			}

			/* Calculate the determinate */
			float det = matrix_determinant(temp);

			/* Fill in the elements of the cofactor */
			output[i][j] = pow(-1.0,i+j+1.0) * det;
		}
	}
	//for (int i=0; i<n-1; i++)
	//	free(c[i]);
	//free(c);
	return output;
}

std::vector< std::vector<float> > matrix_inverse(std::vector< std::vector<float> > input)
{
	std::vector< std::vector<float> > output;
	output = scalar_multiplication(matrix_transpose(matrix_cofactor(input)),1/matrix_determinant(input));
	return output;
}

std::map< std::string, std::vector<float> > maximum_likelihood_classifier( 	std::vector<std::string> species_keys,
																			std::map< std::string, pcl::PointCloud<CanopyInspection::TreeSpectrumAverage> > tree_spectrum_map, 
																			CanopyInspection::TreeSpectrumAverage input_tree_spectrum_average, 
																			std::vector<int> target_bands )
{
	int num_bands = target_bands.size();

	// Compute mean values for each band within each species
	std::map< std::string, std::vector<float> > species_band_mean; 

	for(int i=0; i<species_keys.size(); i++)
	{
		for(int j=0; j<num_bands; j++)
		{
			species_band_mean[species_keys[i]].push_back(0);
			int num_trees = tree_spectrum_map[species_keys[i]].points.size(); 
			for(int k=0; k<num_trees; k++)
			{
				species_band_mean[species_keys[i]][j] += tree_spectrum_map[species_keys[i]].points[k].hyperspectral_data[target_bands[j]];
			}
			species_band_mean[species_keys[i]][j] /= num_trees;
		}
	} 

	for(int i=0; i<num_bands; i++)
	{
		ROS_INFO_STREAM(species_band_mean["PIPA"][i]);
	}

	// Compute covariance matrices between bands within each species 
	std::map< std::string, std::vector< std::vector<float> > > species_covariance_matrices;
	for(int i=0; i<species_keys.size(); i++)
	{
		int num_trees = tree_spectrum_map[species_keys[i]].points.size(); 
		species_covariance_matrices[species_keys[i]] = std::vector< std::vector<float> >(num_bands);
		//species_covariance_matrices[species_keys[i]].reserve(num_bands);
		for(int j=0; j<num_bands; j++)
		{ 
			for(int k=0; k<num_bands; k++)
			{
				species_covariance_matrices[species_keys[i]][j].push_back(0);
//				species_covariance_matrices[species_keys[i]][j][k] = 0;
				for(int m=0; m<num_trees; m++)
				
{					//ROS_INFO_STREAM(i << " " << j << " " << k << " " << m);
					//ROS_INFO_STREAM(target_bands[j] << " " << target_bands[k] << " " << species_keys[i]);
					//ROS_INFO_STREAM(tree_spectrum_map[species_keys[i]].points[m].hyperspectral_data[target_bands[j]] << " " << species_band_mean[species_keys[i]][k]);
					//ROS_INFO_STREAM(tree_spectrum_map[species_keys[i]].points[m].hyperspectral_data[target_bands[k]] << " " << species_band_mean[species_keys[i]][j]);
					float jth_factor = tree_spectrum_map[species_keys[i]].points[m].hyperspectral_data[target_bands[j]] - species_band_mean[species_keys[i]][j];
					float kth_factor = tree_spectrum_map[species_keys[i]].points[m].hyperspectral_data[target_bands[k]] - species_band_mean[species_keys[i]][k];
					//ROS_INFO_STREAM("lkjkljwer");
					species_covariance_matrices[species_keys[i]][j][k] += jth_factor*kth_factor;
				}
				//ROS_INFO_STREAM("flooop" << species_covariance_matrices[species_keys[i]].size() << " "  << species_covariance_matrices[species_keys[i]][j].size());
				species_covariance_matrices[species_keys[i]][j][k] /= num_bands;
			}
			//ROS_INFO_STREAM(j);
		}
		//ROS_INFO_STREAM("    " << i);
	}

	ROS_INFO_STREAM("Finished covariance calculation");

	std::vector< std::vector<float> > target_matrix(num_bands);
	for(int i=0; i<num_bands; i++)
		target_matrix[i].push_back(input_tree_spectrum_average.hyperspectral_data[target_bands[i]]);
	// Probabilities for each species
	std::map<std::string, float> species_probabilities;
	std::cout << "\n\n bands for PIEL";
	for(int i=0; i<tree_spectrum_map["PIEL"].points.size(); i++)
		for(int j=0; j<num_bands; j++)
			std::cout << "\t" << tree_spectrum_map["PIEL"].points[i].hyperspectral_data[target_bands[j]];
	std::cout << "\n";
	for(int i=0; i<species_band_mean["PIEL"].size(); i++)
		std::cout << "\t" << species_band_mean["PIEL"][i];
	std::cout << "\n";
	for(int i=0; i<species_keys.size(); i++)
	{
		float A = pow(2*3.14159,-num_bands/2);
		std::cout << "\n\n" << species_keys[i] << "\n";
		//print_matrix(species_covariance_matrices[species_keys[i]]);
		//ROS_INFO_STREAM(matrix_determinant(species_covariance_matrices[species_keys[i]]));
		//ROS_INFO_STREAM(pow(pow(matrix_determinant(species_covariance_matrices[species_keys[i]]),2),.25));
		//print_matrix(species_covariance_matrices[species_keys[i]]);
		std::cout << matrix_determinant(species_covariance_matrices[species_keys[i]]);
		float B = pow(pow(matrix_determinant(species_covariance_matrices[species_keys[i]]),2),-.25);
		std::vector< std::vector<float> > C = matrix_subtraction(target_matrix,column_matrix(species_band_mean[species_keys[i]]));
		std::vector< std::vector<float> > D = matrix_inverse(species_covariance_matrices[species_keys[i]]);
		std::vector< std::vector<float> > p1 = matrix_multiplication(matrix_transpose(C),D);
		std::vector< std::vector<float> > p2 = matrix_multiplication(p1,(C));
		std::vector< std::vector<float> > p3 = scalar_multiplication(p2,-0.5);
		ROS_INFO_STREAM("size: " << p3.size() << p3[0].size());
		//std::vector< std::vector<float> > E = scalar_multiplication(matrix_multiplication(matrix_multiplication(matrix_transpose(D),E),D),-0.5);
		ROS_INFO_STREAM(A << " " << B);
		//print_matrix(C);
		//print_matrix(D);
		//print_matrix(p1);
		//print_matrix(p2);
		//print_matrix(p3);
		species_probabilities[species_keys[i]] = A*B*exp(p3[0][0]);
		ROS_INFO_STREAM(species_probabilities[species_keys[i]]);	   					  
	}

}



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
		//ROS_INFO_STREAM("bloop");
		current_pixel.crown_id = crown_id;
		getline(data_file, height, ',');
		//ROS_INFO_STREAM(crown_id);
		//ROS_INFO_STREAM(crown_id_map[crown_id]);
		//ROS_INFO_STREAM(tree_pixel_clouds[crown_id_map[crown_id]].size()-1);
		current_pixel.x = tree_pixel_clouds[crown_id_map[crown_id]][tree_pixel_clouds[crown_id_map[crown_id]].size()-1].points.size();
		current_pixel.y = tree_pixel_clouds[crown_id_map[crown_id]].size();
		current_pixel.z = strtof(height.c_str(),0);
		//ROS_INFO_STREAM("bloop");
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


	std::ofstream output_file;
	output_file.open ("species_ordered.csv");
	for(int i=0; i<species_id_list.size(); i++)
	{
		for(int j=0; j<species_tree_averages[species_id_list[i]].points.size(); j++)
		{
			output_file << species_id_list[i] << ", " << species_tree_averages[species_id_list[i]].points[j].crown_id << ", ";
			output_file << species_tree_averages[species_id_list[i]].points[j].min_radius << ", ";
			output_file << species_tree_averages[species_id_list[i]].points[j].max_radius << ", ";
			output_file << species_tree_averages[species_id_list[i]].points[j].min_height << ", ";
			output_file << species_tree_averages[species_id_list[i]].points[j].max_height << ", ";
			output_file << species_tree_averages[species_id_list[i]].points[j].avg_height << ", ";
			for(int k=0; k<426; k++)
				output_file << species_tree_averages[species_id_list[i]].points[j].hyperspectral_data[k] << ", ";
			output_file << "\n";
		}
	}
	output_file.close();

	//std::vector< std::vector<float> > matrix(4);
	//matrix[0].push_back(1); matrix[0].push_back(2); matrix[0].push_back(2); matrix[0].push_back(4);
	//matrix[1].push_back(2); matrix[1].push_back(3); matrix[1].push_back(3); matrix[1].push_back(4);
	//matrix[2].push_back(3); matrix[2].push_back(2); matrix[2].push_back(3); matrix[2].push_back(3);
	//matrix[3].push_back(4); matrix[3].push_back(2); matrix[3].push_back(3); matrix[3].push_back(4);
	//ROS_INFO_STREAM("Input:");
	//print_matrix(matrix);
	//ROS_INFO_STREAM("Output:");
	//print_matrix(matrix_inverse(matrix));
	//ros::Duration(5).sleep();
	//return -1;

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

	maximum_likelihood_classifier(species_id_list, species_tree_averages, species_tree_averages[target_species].points[tree_index], target_bands);

	while(ros::ok())
	{
		ros::Duration(0.5).sleep();
	}
}


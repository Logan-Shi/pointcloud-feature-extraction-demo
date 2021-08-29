#ifndef WELDSEAM_MEASURE_H
#define WELDSEAM_MEASURE_H
#include <iostream>
#include <fstream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/don.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/covariance_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/boundary.h>

#include <pcl/search/impl/search.hpp>
#ifndef PCL_NO_PRECOMPILE
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_types.h>
PCL_INSTANTIATE(Search, PCL_POINT_TYPES)
#endif // PCL_NO_PRECOMPILE

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;

double calc_circle(const PointCloudT::Ptr cloud_boundary, pcl::ModelCoefficients::Ptr coefficients_circle, double percentage, double diameter, double buffer, double threshold, int iterations);
void calc_boundary(const PointCloudT::Ptr cloud_p, PointCloudT::Ptr cloud_boundary, double radius_search_small, double radius_search_large, double angle_threshold);
double calc_plane(const PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_p, pcl::ModelCoefficients::Ptr coefficients, double plane_threshold,double crop_size, double x_pos, double y_pos, int iterations);
PointCloudT::Ptr crop_box(const PointCloudT::Ptr cloud_in, double box_size, double x_pos, double y_pos);

#endif
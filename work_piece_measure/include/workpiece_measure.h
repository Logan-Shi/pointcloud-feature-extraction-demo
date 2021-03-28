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
#include <pcl/features/don.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/filters/covariance_sampling.h>
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

bool next_iteration = false;
bool save_tf = false;
ofstream results;
pcl::console::TicToc pcl_timer;
int iterations = 1;
double diameter = 4;
double buffer = 0.5;
double z_min = -500;
double z_max = 500;
double threshold = 0.01;
double radius_search_small = 0.5;
double radius_search_large = 0.5;
double angle_threshold = 4;
pcl::visualization::PCLVisualizer viewer ("RANSAC demo");

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;

  if (event.getKeySym () == "s" && event.keyDown ())
    save_tf = true;
}

double calc_circle(const PointCloudT::Ptr cloud_boundary, pcl::ModelCoefficients::Ptr coefficients_circle, double diameter, double buffer, double threshold);
void calc_boundary(const PointCloudT::Ptr cloud_p, PointCloudT::Ptr cloud_boundary,double,double,double);
double calc_plane(const PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_p,double z_min,double z_max);
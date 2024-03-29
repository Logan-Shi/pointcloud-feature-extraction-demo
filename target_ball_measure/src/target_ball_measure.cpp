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

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/ublas/assignment.hpp>
#include "bayesopt/bayesopt.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;
bool save_tf = false;
ofstream results;

// The point clouds we will be using
PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
int iterations = 1;
double distanceThreshold = 0.001;
int need_viewer = 0;

class ExampleOneD: public bayesopt::ContinuousModel
{
public:
  ExampleOneD(bayesopt::Parameters par):
    ContinuousModel(1,par) {}

  double evaluateSample(const vectord& xin)
  {
    if (xin.size() != 1)
      {
  std::cout << "WARNING: This only works for 1D inputs." << std::endl
      << "WARNING: Using only first component." << std::endl;
      }

    double distanceThreshold = xin(0);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setRadiusLimits(14, 16);
    seg.setMaxIterations (iterations);
  
    seg.setInputCloud (cloud_in);
    // time.tic ();
    seg.segment (*inliers, *coefficients);
    // double percentage = double(inliers->indices.size())/cloud_in->size();
    // double time_consumption = time.toc();
    // std::cout << "Applied "<<std::to_string(iterations) << " ransac iterations in " << time_consumption << " ms" << std::endl;
    double radius = coefficients->values[3];
    return std::abs(radius-15);
  };

  bool checkReachability(const vectord &query)
  {return true;};

  void printOptimal()
  {
    std::cout << "Optimal:" << 0.23719 << std::endl;
  }
};

void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;

  if (event.getKeySym () == "s" && event.keyDown ())
    save_tf = true;
}

int
main (int argc,
      char* argv[])
{
  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file_in.ply iterations\n", argv[0]);
    PCL_ERROR ("Provide the ply file and iterations.\n");
    return (-1);
  }

  pcl::console::TicToc time;
  time.tic ();
  if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

  if (argc > 2)
  {
    iterations = atoi(argv[2]);
    distanceThreshold = strtod(argv[3],NULL);
    need_viewer = atoi(argv[4]);
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1.\n");
      return (-1);
    }
  }

  if (!need_viewer)
  {
    bayesopt::Parameters parameters;
    // if(nargs > 1){
    //   if(!bayesopt::utils::ParamLoader::load(args[1], parameters)){
    //       std::cout << "ERROR: provided file \"" << args[1] << "\" does not exist" << std::endl;
    //       return -1;
    //   }
    // }
    // else{
    parameters.n_init_samples = 10;
    parameters.n_iterations = 300;
  
    parameters.crit_name = "cHedge(cEI,cLCB,cExpReturn,cOptimisticSampling)";
      //bayesopt::utils::ParamLoader::save("bo_oned.txt", parameters);
    // }
    
    ExampleOneD opt(parameters);
    vectord result(1);
    opt.optimize(result);
    
    std::cout << "Result:" << result << std::endl;
  
  }
  else
  {
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_SPHERE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setRadiusLimits(14, 16);
    seg.setMaxIterations (iterations);
  
    seg.setInputCloud (cloud_in);
    // time.tic ();
    seg.segment (*inliers, *coefficients);
    double percentage = double(inliers->indices.size())/cloud_in->size();
    double time_consumption = time.toc();
    std::cout << "Applied "<<std::to_string(iterations) << " ransac iterations in " << time_consumption << " ms" << std::endl;

    results.open("result/test.txt", std::ios_base::app);
    results << time_consumption << ", "<< percentage << ", " << iterations << ", " << distanceThreshold << ", "<< coefficients->values[3]<<";\n";
    results.close();
    pcl::ModelCoefficients sphere_coeff;
    sphere_coeff.values.resize(4);
    sphere_coeff.values[0] = coefficients->values[0]; //x
    sphere_coeff.values[1] = coefficients->values[1]; //y
    sphere_coeff.values[2] = coefficients->values[2]; //z
    sphere_coeff.values[3] = coefficients->values[3]; //radius
  
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("target ball demo");
  
    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
  
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 180,20,20);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in");
    viewer.addSphere(sphere_coeff);
  
    // Adding text descriptions in each viewport
    viewer.addText ("Red: Original point cloud\nWhite: Ransac result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");
  
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "RANSAC iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 50, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
    viewer.addText ("target ball radius: " + std::to_string(coefficients->values[3])+"\n", 10, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "radius");
    viewer.addText ("Percentage of inliers: " + std::to_string(percentage) + "\n", 10, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "inliers percentage");
  
    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
  
    // Set camera position and orientation
    viewer.setCameraPosition (0, 0, 200, 0, 0, 0, 0);
    viewer.addCoordinateSystem(10);
    viewer.setSize (1280, 1024);  // Visualiser window size
  
    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);
    
    // Display the visualiser
    while (!viewer.wasStopped ())
    {
      viewer.spinOnce ();
  
      // The user pressed "space" :
      if (next_iteration)
      {
        PCL_ERROR ("\nBad result, next sample.\n");
        return (-1);
      }
      next_iteration = false;
  
      if (save_tf)
      {
        results.open("result/test.txt", std::ios_base::app);
        // results << "sphere is positioned at: (in " << argv[1]<<" frame)\n";
        // results << "  Translation vector :\n";
        results << sphere_coeff.values[0]<< ", " << sphere_coeff.values[1] << ", " << sphere_coeff.values[2]<<";\n";
        results.close();
        // std::cout<<"matching finished,exiting...\n";
        return(0);
      }
    }
  }
  return (0);
}
#include <workpiece_measure.h>
pcl::visualization::PCLVisualizer viewer ("RANSAC demo");

bool next_iteration = false;
bool save_tf = false;
ofstream results;
pcl::console::TicToc pcl_timer;
int iterations = 1;
double diameter = 4;
double buffer = 0.5;
double plane_th = 0.01;
double crop_size = 50;
double threshold = 0.01;
double radius_search_small = 0.5;
double radius_search_large = 0.5;
double angle_threshold = 4;

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;

  if (event.getKeySym () == "s" && event.keyDown ())
    save_tf = true;
}

int main (int argc, char* argv[])
{
  PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
  // Checking program arguments
  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s file_in.ply iterations\n", argv[0]);
    PCL_ERROR ("Provide the ply file and iterations.\n");
    return (-1);
  }

  pcl_timer.tic ();
  if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }
  std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << pcl_timer.toc () << " ms" << std::endl;

  // double threshold = 0.01;
  if (argc > 2)
  {
    iterations = atoi(argv[2]);
    diameter = std::stod(argv[3]);
    buffer = std::stod(argv[4]);
    plane_th = std::stod(argv[5]);
    crop_size = std::stod(argv[6]);
    threshold = std::stod(argv[7]);
    radius_search_small = std::stod(argv[8]);
    radius_search_large = std::stod(argv[9]);
    angle_threshold = std::stod(argv[10]);
    std::cout<<"iterations: "<<iterations<<"\n";
    std::cout<<"diameter: "<<diameter<<"\n";
    std::cout<<"buffer: "<<buffer<<"\n";
    std::cout<<"plane_th: "<<plane_th<<"\n";
    std::cout<<"crop_size: "<<crop_size<<"\n";
    std::cout<<"threshold: "<<threshold<<"\n";
    std::cout<<"radius_search_small: "<<radius_search_small<<"\n";
    std::cout<<"radius_search_large: "<<radius_search_large<<"\n";
    std::cout<<"angle_threshold: "<<angle_threshold<<"\n";
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1.\n");
      return (-1);
    }
  }
  // ---------------------------------------------------------------------------------------------------------
  PointCloudT::Ptr cloud_p (new PointCloudT);
  pcl_timer.tic ();
  PointCloudNT::Ptr normals(new PointCloudNT); 
  calc_plane(cloud_in, cloud_p, plane_th, crop_size,iterations, normals);
  std::cout << "Plane ransaced in "<< pcl_timer.toc () << " ms" << std::endl;

  PointCloudT::Ptr cloud_boundary (new PointCloudT);
  pcl_timer.tic ();
  calc_boundary(cloud_p, cloud_boundary, normals, radius_search_small,radius_search_large,angle_threshold);
  std::cout << "Boundary extracted in "<< pcl_timer.toc () << " ms" << std::endl;

  double percentage = 0;
  pcl_timer.tic ();
  pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
  percentage = calc_circle(cloud_boundary, coefficients_circle, percentage, diameter, buffer, threshold,iterations);
  std::cout<<"percentage: "<<percentage<<"\n";
  percentage = calc_circle(cloud_boundary, coefficients_circle, percentage, 9, buffer, threshold,iterations);
  std::cout<<"percentage: "<<percentage<<"\n";
  percentage = calc_circle(cloud_boundary, coefficients_circle, percentage, 26, buffer, threshold,iterations);
  std::cout<<"percentage: "<<percentage<<"\n";
  std::cout << "circle extracted in "<< pcl_timer.toc () << " ms" << std::endl;

  // ---------------------------------------------------------------------------------------------------------
  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255,255,255);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_p_color_h (cloud_p, 180,20,20);
  viewer.addPointCloud (cloud_p, cloud_p_color_h, "cloud_p");

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_boundary_color_h (cloud_boundary, 20,180,20);
  viewer.addPointCloud (cloud_boundary, cloud_boundary_color_h, "cloud_boundary");

  pcl::ModelCoefficients cylinder_coeff;
  double cylinder_length = 15;
  cylinder_coeff.values.resize (7);    // We need 7 values
  cylinder_coeff.values[0] = coefficients_circle->values[0];
  cylinder_coeff.values[1] = coefficients_circle->values[1];
  cylinder_coeff.values[2] = coefficients_circle->values[2];
 
  cylinder_coeff.values[3] = cylinder_length*coefficients_circle->values[4];
  cylinder_coeff.values[4] = cylinder_length*coefficients_circle->values[5];
  cylinder_coeff.values[5] = cylinder_length*coefficients_circle->values[6];

  cylinder_coeff.values[6] = coefficients_circle->values[3];
 
  viewer.addCylinder (cylinder_coeff);
  // viewer.addCoordinateSystem(10,coefficients_circle->values[0],coefficients_circle->values[1],coefficients_circle->values[2]);

  // Adding text descriptions in each viewport
  viewer.addText ("White: Original point cloud\nRed: Ransac Plane result\nGreen: Boundary result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");

  // std::stringstream ss;
  // ss << iterations;
  // std::string iterations_cnt = "RANSAC iterations = " + ss.str ();
  // viewer.addText (iterations_cnt, 10, 50, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
  viewer.addText ("hole radius: " + std::to_string(coefficients_circle->values[3])+"\n", 10, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "radius");
  // viewer.addText ("Percentage of inliers: " + std::to_string(percentage) + "\n", 10, 90, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "inliers percentage");

  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_boundary");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"cloud_in");
  
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
      // results << "circle is positioned at: (in " << argv[1]<<" frame)\n";
      // results << "  Translation vector :\n";
      results << coefficients_circle->values[0]<< ", " << coefficients_circle->values[1] << ", " << coefficients_circle->values[2]<<";\n";
      results.close();
      std::cout<<"matching finished,exiting...\n";
      return(0);
    }
  }
  return (0);
}
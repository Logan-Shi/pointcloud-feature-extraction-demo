#include <workpiece_measure.h>

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
    z_min = std::stod(argv[5]);
    z_max = std::stod(argv[6]);
    threshold = std::stod(argv[7]);
    radius_search_small = std::stod(argv[8]);
    radius_search_large = std::stod(argv[9]);
    angle_threshold = std::stod(argv[10]);
    std::cout<<"iterations: "<<iterations<<"\n";
    std::cout<<"diameter: "<<diameter<<"\n";
    std::cout<<"buffer: "<<buffer<<"\n";
    std::cout<<"z_min: "<<z_min<<"\n";
    std::cout<<"z_max: "<<z_max<<"\n";
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
  calc_plane(cloud_in, cloud_p, z_min, z_max);

  PointCloudT::Ptr cloud_boundary (new PointCloudT);
  calc_boundary(cloud_p, cloud_boundary, radius_search_small,radius_search_large,angle_threshold);

  pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
  calc_circle(cloud_boundary, coefficients_circle, diameter, buffer, threshold);
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
  // viewer.addText ("target ball radius: " + std::to_string(coefficients->values[3])+"\n", 10, 70, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "radius");
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

double calc_circle(const PointCloudT::Ptr cloud_boundary, pcl::ModelCoefficients::Ptr coefficients_circle, double diameter, double buffer, double threshold)
{
  std::cout << "input size:  "<<std::to_string(cloud_boundary->size()) <<"" << std::endl;
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg_circle;
  // Optional
  seg_circle.setOptimizeCoefficients (true);
  // Mandatory
  seg_circle.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg_circle.setMethodType (pcl::SAC_RANSAC);
  seg_circle.setDistanceThreshold (threshold);
  double radius_min = (diameter-buffer)/2;
  double radius_max = (diameter+buffer)/2;
  std::cout<<"radius_min: "<<radius_min<<"\n";
  std::cout<<"radius_max: "<<radius_max<<"\n";
  seg_circle.setRadiusLimits(radius_min, radius_max);
  seg_circle.setMaxIterations (iterations);

  pcl::ExtractIndices<PointT> extract_circle;
  seg_circle.setInputCloud (cloud_boundary);
  // pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
  
  pcl_timer.tic ();
  seg_circle.segment (*inliers_circle, *coefficients_circle);
  // if (inliers_circle->indices.size() == 0 )
  // {
  //   PCL_ERROR("\n can not extract circle from given boundaries\n");
  // }
  // extract_circle.setInputCloud(cloud_boundary);
  // extract_circle.setIndices(inliers_circle);
  // extract_circle.setNegative(false);
  // PointCloudT::Ptr cloud_circle (new PointCloudT);  // Plane point cloud
  // extract_circle.filter(*cloud_circle);
  std::cout << "Applied "<<std::to_string(iterations) << " circle ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
  std::cout << "Circle size:  "<<std::to_string(inliers_circle->indices.size()) <<"" << std::endl;

  double percentage = double(inliers_circle->indices.size())/cloud_boundary->size();

  std::cout<<"circle percentage: "<<percentage<<"\n";
  std::cout<<"circle radius: "<<coefficients_circle->values[3]<<"\n";
  return percentage;
}

void calc_boundary(const PointCloudT::Ptr cloud_p, PointCloudT::Ptr cloud_boundary, double radius_search_small, double radius_search_large, double angle_threshold)
{
  // Boundary
  pcl::PointCloud<pcl::Boundary> boundaries; 
  pcl::BoundaryEstimation<PointT, PointNT, pcl::Boundary> boundEst; 
  pcl::NormalEstimation<PointT, PointNT> normEst; 
  PointCloudNT::Ptr normals(new PointCloudNT); 
  normEst.setInputCloud(cloud_p); 
  normEst.setRadiusSearch(radius_search_small); 
  pcl_timer.tic();
  normEst.compute(*normals); 
  std::cout << "Calculated normals in " << pcl_timer.toc () << " ms" << std::endl;
 
  boundEst.setInputCloud(cloud_p);
  boundEst.setInputNormals(normals);
  boundEst.setRadiusSearch(radius_search_large); 
  boundEst.setAngleThreshold(M_PI/angle_threshold); 
  boundEst.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>)); 
  pcl_timer.tic();
  boundEst.compute(boundaries); 
  std::cout << "Calculated boundaries in " << pcl_timer.toc () << " ms" << std::endl;
 
  for(int i = 0; i < cloud_p->points.size(); i++) 
  {
    if(boundaries[i].boundary_point > 0) 
    { 
      cloud_boundary->push_back(cloud_p->points[i]); 
    }
  }
}

double calc_plane(const PointCloudT::Ptr cloud_in, PointCloudT::Ptr cloud_p,double z_min,double z_max)
{
  double percentage = 0;
  PointCloudT::Ptr cloud_filtered (new PointCloudT);
  pcl::ConditionAnd<PointT>::Ptr range_cond(new pcl::ConditionAnd<PointT> ());
  range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::GT,z_min)));
  range_cond->addComparison(pcl::FieldComparison<PointT>::ConstPtr (new pcl::FieldComparison<PointT>("z",pcl::ComparisonOps::LT,z_max)));
  pcl::ConditionalRemoval<PointT> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(false);
  condrem.filter(*cloud_filtered);

  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  // seg.setRadiusLimits(14, 16);
  seg.setMaxIterations (iterations);

  pcl::ExtractIndices<PointT> extract;
  seg.setInputCloud (cloud_filtered);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  
  pcl_timer.tic ();
  seg.segment (*inliers, *coefficients);
  if (inliers->indices.size() == 0 )
  {
    return percentage;
  }
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  
  extract.filter(*cloud_p);
  std::cout << "Applied "<<std::to_string(iterations) << " plane ransac iterations in " << pcl_timer.toc () << " ms" << std::endl;
  std::cout << "Plane size:  "<<std::to_string(inliers->indices.size()) <<"" << std::endl;

  percentage = double(inliers->indices.size())/cloud_filtered->size();
  return percentage;
}
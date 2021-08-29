#include <weldseam_measure.h>
pcl::visualization::PCLVisualizer viewer ("RANSAC demo");
bool next_iteration = false;
bool save_tf = false;
ofstream results;
pcl::console::TicToc pcl_timer;
bool raw_view = 0;
int iterations = 100;
double x_pos = -50;
double y_pos = 0;
double plane_th = 0.01;
double crop_size = 50;
double threshold = 0.01;
double neighbor_size = 0.5;
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
    raw_view = atoi(argv[2]);
    x_pos = std::stod(argv[3]);
    y_pos = std::stod(argv[4]);
    plane_th = std::stod(argv[5]);
    crop_size = std::stod(argv[6]);
    threshold = std::stod(argv[7]);
    neighbor_size = std::stod(argv[8]);
    radius_search_large = std::stod(argv[9]);
    angle_threshold = std::stod(argv[10]);
    std::cout<<"raw_view: "<<raw_view<<"\n";
    std::cout<<"x_pos: "<<x_pos<<"\n";
    std::cout<<"y_pos: "<<y_pos<<"\n";
    std::cout<<"plane_th: "<<plane_th<<"\n";
    std::cout<<"crop_size: "<<crop_size<<"\n";
    std::cout<<"threshold: "<<threshold<<"\n";
    std::cout<<"neighbor_size: "<<neighbor_size<<"\n";
    std::cout<<"radius_search_large: "<<radius_search_large<<"\n";
    std::cout<<"angle_threshold: "<<angle_threshold<<"\n";
    if (iterations < 1)
    {
      PCL_ERROR ("Number of initial iterations must be >= 1.\n");
      return (-1);
    }
  }

  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  if (raw_view)
  {
    PointCloudT::Ptr cloud_filtered (new PointCloudT);
    pcl::VoxelGrid<PointT> downsampler;
    downsampler.setInputCloud (cloud_in);
    downsampler.setLeafSize (0.1f, 0.1f, 0.1f);
    downsampler.filter (*cloud_filtered);
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_color_h (cloud_filtered, 255,255,255);
    viewer.addPointCloud (cloud_filtered, cloud_filtered_color_h, "cloud_in");
  }
  else{
    PointCloudT::Ptr cloud_p (new PointCloudT);
    PointCloudT::Ptr cloud_filtered (new PointCloudT);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl_timer.tic ();

    calc_plane(cloud_in, cloud_p, coefficients, plane_th, crop_size, x_pos, y_pos, iterations);
    std::cout << "Plane ransaced in "<< pcl_timer.toc () << " ms" << std::endl;

    pcl::StatisticalOutlierRemoval<PointT> sorfilter (true); // Initializing with true will allow us to extract the removed indices
    sorfilter.setInputCloud (cloud_p);
    sorfilter.setMeanK (neighbor_size);
    sorfilter.setStddevMulThresh (threshold);
    sorfilter.filter (*cloud_filtered);
    // The resulting cloud_out contains all points of cloud_in that have an average distance to their 8 nearest neighbors that is below the computed threshold
    // Using a standard deviation multiplier of 1.0 and assuming the average distances are normally distributed there is a 84.1% chance that a point will be an inlier
    std::cout << "plane params: \n";
    std::cout << coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<", " << coefficients->values[3]<<"\n";
    
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);
    std::cout<<"centroid coordinate: \n";
    std::cout << centroid[0]<< ", " << centroid[1] << ", " << centroid[2]<<", " << centroid[3]<<"\n";
    double distance = 0;
    distance = coefficients->values[0]*centroid[0]+
               coefficients->values[1]*centroid[1]+
               coefficients->values[2]*centroid[2]+
               coefficients->values[3]*centroid[3];
    std::cout << "distance is "<<distance<<"\n";
    // ---------------------------------------------------------------------------------------------------------
    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_filtered_color_h (cloud_filtered, 180,20,20);
    viewer.addPointCloud (cloud_filtered, cloud_filtered_color_h, "cloud_filtered");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_p_color_h (cloud_p, 255,255,255);
    viewer.addPointCloud (cloud_p, cloud_p_color_h, "cloud_p");

    // pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_boundary_color_h (cloud_boundary, 20,180,20);
    // viewer.addPointCloud (cloud_boundary, cloud_boundary_color_h, "cloud_boundary");

    // Adding text descriptions in each viewport
    viewer.addText ("White: Original point cloud\nRed: Ransac Plane result\nGreen: Boundary result", 10, 10, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "ransac_info");

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,0.5,"cloud_in");

  }
  
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
      // results << coefficients->values[0]<< ", " << coefficients->values[1] << ", " << coefficients->values[2]<<", " << coefficients->values[3]";\n";
      results.close();
      std::cout<<"matching finished,exiting...\n";
      return(0);
    }
  }
  return (0);
}
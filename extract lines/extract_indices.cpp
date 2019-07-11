#include <iostream>
#include <math.h>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZ Point;

void Visualizer(pcl::ModelCoefficients::Ptr coefficients,
		pcl::visualization::PCLVisualizer &viewer, const int distance,
		std::string line_id){
    //Add point cloud and the corresponding line to viewer.

    //Calculate a rough begin point and end point. As dir_z is too small, we estimate it into a 2d line with dir_z = 0.
    Point mid(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float dir_x = coefficients->values[3];
    float dir_y = coefficients->values[4];
    float dir_z = coefficients->values[5];
    float x_y = dir_x/dir_y;
    double angle = atan(x_y);
    Point begin(mid.x-distance*sin(angle), mid.y-distance*cos(angle), mid.z);
    Point end(mid.x+distance*sin(angle), mid.y+distance*cos(angle), mid.z);

    viewer.addLine(begin, end, 255, 0, 0,  line_id);
    std::cout<<"Line added\n";
}




int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    std::string pcd_file;
    pcl::visualization::PCLVisualizer viewer ("Visualize Line");
    viewer.setSize (800, 600);

    if (argc < 2)
    {
      printf ("\nUsage: ./planar_segmentation PCDfile\n\n");
      exit (0);
    }
    pcd_file = argv[1];

  // Fill in the cloud data
  pcl::PCDReader reader;
  reader.read (pcd_file, *cloud_blob);


  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;



  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("map004_ground_downsampled.pcd", *cloud_filtered, false);

  viewer.addPointCloud<Point> (cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.1);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointCloud<pcl::PointXYZ> merge_all;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);

  
//    Extract Inliers(Output points representing the line:


    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    if(i<=3){
	//Extract the first three lines
        std::string line_id = "line"+std::to_string(i);
	Visualizer(coefficients, viewer, 100, line_id);
    }
/*  
    Save as points.
    std::stringstream ss;
    ss << "map004_line_" << i << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);

*/

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
  viewer.spin();
  return (0);
}

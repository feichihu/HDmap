#include <iostream>
#include <math.h>
#include <vector>
#include <string>
#include <algorithm>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointXYZ Point2;

float difference(std::vector<float> &lane1,std::vector<float> &lane2){
    Point2 mid1(lane1[0], lane1[1], lane1[2]);
    Point2 mid2(lane2[0], lane2[1], lane2[2]);
    float d2 = lane2[3]*mid2.x + lane2[4]*mid2.y + lane2[5]*mid2.z;
    float d1 = lane1[3]*mid1.x + lane1[4]*mid1.y + lane1[5]*mid1.z;
    return ((mid1.x + mid1.y) - (mid2.x + mid2.y));

}



bool comp(std::vector<float> &lane1,std::vector<float> &lane2)
{
    float diff = difference(lane1, lane2);
    return diff > 0;
}



void Visualizer(std::vector<std::vector<float>> &all_lines,
		pcl::visualization::PCLVisualizer &viewer, const int distance,
		std::string line_id, 
		const char color){
    //Add point cloud and the corresponding line to viewer.

    //Calculate a rough begin point and end point. As dir_z is too small, we estimate it into a 2d line with dir_z = 0.
    for(int i = 0; i < all_lines.size(); i++){
    Point2 mid(all_lines[i][0], all_lines[i][1], all_lines[i][2]);
    float dir_x = all_lines[i][3];
    float dir_y = all_lines[i][4];
    float x_y = dir_x/dir_y;
    double angle = atan(x_y);
    Point2 begin(mid.x-distance*sin(angle), mid.y-distance*cos(angle), mid.z);
    Point2 end(mid.x+distance*sin(angle), mid.y+distance*cos(angle), mid.z);

    double r = 0, g = 0, b = 0;
    if(color == 'r')r = 255;
    else if(color == 'g') g = 255;
    else b = 255;
    viewer.addLine(begin, end, r, g, b, line_id + "_" + std::to_string(i));
    viewer.addText3D (line_id + std::to_string(i), begin, 1.0, 0, 255, 0, line_id + std::to_string(i));
    std::cout<<"Line " << i << " added\n";
    }
}


void generate_trajectory(std::vector<std::vector<float> > &all_lines,
			std::vector<std::vector<float> > &all_trajectory){
	for(int i = 0; i < all_lines.size()-1; i++){
		//Nearly parallel...
		std::vector<float> current_trajectory(all_lines[i]);
		float mid_x = (all_lines[i][0] + all_lines[i+1][0])/2;
		float mid_y = (all_lines[i][1] + all_lines[i+1][1])/2;
		float mid_z = (all_lines[i][2] + all_lines[i+1][2])/2;
		current_trajectory[0] = mid_x;
		current_trajectory[1] = mid_y;
		current_trajectory[2] = mid_z;
		all_trajectory.push_back(current_trajectory);
	}
}
			



int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<Point>::Ptr cloud_filtered (new pcl::PointCloud<Point>), cloud_p (new pcl::PointCloud<Point>), cloud_f (new pcl::PointCloud<Point>);
    
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
  writer.write<Point> ("map004_ground_downsampled.pcd", *cloud_filtered, false);

  viewer.addPointCloud<Point> (cloud_filtered, "cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,"cloud");

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<Point> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_LINE);
  seg.setMethodType (pcl::SAC_PROSAC);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.1);

  // Create the filtering object
  pcl::ExtractIndices<Point> extract;
  pcl::PointCloud<Point> merge_all;

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  std::vector<std::vector<float> > all_lines;

  // While 50% of the original cloud is still there
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

    if(i<=10){
	//Extract the first eight lines
	bool duplicate = false;
        //std::string line_id = "line"+std::to_string(i);
        std::vector<float> current_line(coefficients->values);
	for(int i = 0; i < all_lines.size(); i++){
		float diff = difference(all_lines[i], current_line);
		std::cout<<all_lines[i][0]<<" "<<all_lines[i][1]<<" "<<
			all_lines[i][2]<<" dir_x:"<<all_lines[i][3]<<" dir_y: "
			<<all_lines[i][4]<<" "<<diff<<std::endl;
		if(abs(diff) <= 1.0 || abs(diff) >= 24 ){
			duplicate = true;
			break;
		}

	}
	if(!duplicate){
		all_lines.push_back(current_line);	
		//Visualizer(coefficients, viewer, 80, line_id);
	}
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

  std::sort(all_lines.begin(), all_lines.end(), comp);
  std::vector<std::vector<float>> all_trajectory;
  generate_trajectory(all_lines, all_trajectory);
  Visualizer(all_lines, viewer, 80, "L", 'r');
  Visualizer(all_trajectory, viewer, 80, "TR", 'b');
  
  viewer.spin();
  return (0);
}

#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointXYZ Point2;

void remove(const uint8_t r, const uint8_t g, const uint8_t b, pcl::PointCloud<Point>::Ptr p_obstacles){
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<Point> extract;
  for (int i = 0; i < (*p_obstacles).size(); i++)
  {
    if (p_obstacles->points[i].r < r && p_obstacles->points[i].g < g &&
	p_obstacles->points[i].b > b) // e.g. reserve all blue points
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(p_obstacles);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*p_obstacles);
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
  pcl::PointCloud<Point>::Ptr cloud_filtered (new pcl::PointCloud<Point>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;

  if (argc < 3)
    {
      printf ("\nUsage: ./planar_segmentation PCDfile savedPCDfile\n\n");
      exit (0);
    }

  std::string pcdfile = argv[1];
  std::string saved_file = argv[2];
  // Replace the path below with the path where you saved your file
  reader.read<Point> (pcdfile, *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << (*cloud).size() << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<Point> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << (*cloud_filtered).size() << std::endl;

  //remove(128, 128, 128, cloud_filtered);//Trying to extract roads using rgb value

  pcl::PCDWriter writer;
  writer.write<Point> (saved_file, *cloud_filtered, false);

  std::cerr << "Ground cloud after removing: " << std::endl;
  std::cerr << (*cloud_filtered).size() << std::endl;

  pcl::visualization::CloudViewer viewer("Cloud Viewer");
  viewer.showCloud(cloud_filtered);
  while (!viewer.wasStopped ())
  {
  //you can also do cool processing here
  //FIXME: Note that this is running in a separate thread from viewerPsycho
  //and you should guard against race conditions yourself...
  //user_data++;
  }

  // Extract non-ground returns
  /*extract.setNegative (true);
  extract.filter (*cloud_filtered);

  std::cerr << "Object cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);
*/
  return (0);
}


#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

void remove(const int threshold_y1, pcl::PointCloud<pcl::PointXYZ>::Ptr p_obstacles){
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  for (int i = 0; i < (*p_obstacles).size(); i++)
  {
    pcl::PointXYZ pt(p_obstacles->points[i].x, p_obstacles->points[i].y, p_obstacles->points[i].z);
    if (pt.y < threshold_y1) // e.g. remove all pts below zAvg
    {
      inliers->indices.push_back(i);
    }
  }
  extract.setInputCloud(p_obstacles);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*p_obstacles);
}


int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("map004_downsample.pcd", *cloud);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << (*cloud).size() << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered);

  std::cerr << "Ground cloud after filtering: " << std::endl;
  std::cerr << (*cloud_filtered).size() << std::endl;

  remove(-80, -30, cloud_filtered);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("map004_ground.pcd", *cloud_filtered, false);

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


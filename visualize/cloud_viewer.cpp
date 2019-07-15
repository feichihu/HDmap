#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int user_data;
typedef pcl::PointXYZRGBA Point;

void
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
    pcl::PointXYZ o;
    o.x = 1.0;
    o.y = 0;
    o.z = 0;
    viewer.addSphere (o, 0.25, "sphere", 0);
    std::cout << "i only run once" << std::endl;

}

void
viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}

int
main (int argc, char *argv[])
{
    std::string pcd_file, file_3dm;

    if (argc < 2)
    {
      printf ("\nUsage: ./cloud_viewer PCDfile\n\n");
      exit (0);
    }
    pcd_file = argv[1];

    pcl::visualization::PCLVisualizer viewer ("Visualize Line");
    viewer.setSize (800, 600);

    printf ("  loading %s\n", pcd_file.c_str ());
    pcl::PointCloud<Point>::Ptr cloud (new pcl::PointCloud<Point>);
    pcl::PCLPointCloud2 cloud2;


    if (pcl::io::loadPCDFile (pcd_file, cloud2) == -1)
      throw std::runtime_error ("  PCD file not found.");

   // pcl::PointXYZ p1(-300, -70, 10);
   // pcl::PointXYZ p2(-200, 0, 10);
    fromPCLPointCloud2 (cloud2, *cloud);
  //  pcl::visualization::PointCloudColorHandlerCustom<Point> handler (cloud, 0, 255, 0);
    viewer.addPointCloud<Point> (cloud);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    printf ("  %lu points in data set\n", cloud->size ());
  //  viewer.addLine(p1, p2, 255, 0, 0, "line");
    viewer.spin();
    return 0;
}

#include <iostream>

#include <boost/thread/thread.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/file_io.h>
#include <pcl/io/ply/ply_parser.h>
#include <pcl/io/ply/ply.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/project_inliers.h>

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv){

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PolygonMesh cl;
  std::vector<int> filenames;
  bool file_is_pcd = false;
  bool file_is_ply = false;
  bool file_is_txt = false;
  bool file_is_xyz = false;



  pcl::console::TicToc tt;
  pcl::console::print_highlight ("Loading ");

  filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
  if(filenames.size()<=0){
      filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
      if(filenames.size()<=0){
          filenames = pcl::console::parse_file_extension_argument(argc, argv, ".txt");
          if(filenames.size()<=0){
              filenames = pcl::console::parse_file_extension_argument(argc, argv, ".xyz");
              if(filenames.size()<=0){

                  return -1;
              }else if(filenames.size() == 1){
                  file_is_xyz = true;
              }
          }else if(filenames.size() == 1){
             file_is_txt = true;
        }
    }else if(filenames.size() == 1){
          file_is_pcd = true;
    }
  }
  else if(filenames.size() == 1){
      file_is_ply = true;
  }else{

      return -1;
  }

  std::string output_dir;
  if(pcl::console::parse_argument(argc, argv, "-o", output_dir) == -1){
      PCL_ERROR ("Need an output directory! Please use <input cloud> -o <output dir>\n");
      return(-1);
  }

  if(file_is_pcd){
      if(pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0){
              std::cout << "Error loading point cloud " << argv[filenames[0]]  << "\n";
              return -1;
      }
      pcl::console::print_info("\nFound pcd file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
    }else if(file_is_ply){
      pcl::io::loadPLYFile(argv[filenames[0]],*cloud);
      if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
          pcl::console::print_warn("\nloadPLYFile could not read the cloud, attempting to loadPolygonFile...\n");
          pcl::io::loadPolygonFile(argv[filenames[0]], cl);
          pcl::fromPCLPointCloud2(cl.cloud, *cloud);
          if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
              pcl::console::print_warn("\nloadPolygonFile could not read the cloud, attempting to PLYReader...\n");
              pcl::PLYReader plyRead;
              plyRead.read(argv[filenames[0]],*cloud);
              if(cloud->points.size()<=0 or cloud->points[0].x<=0 and cloud->points[0].y<=0 and cloud->points[0].z<=0){
                  pcl::console::print_error("\nError. ply file is not compatible.\n");
                  return -1;
              }
          }
       }

      pcl::console::print_info("\nFound ply file.");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");

    }else if(file_is_txt or file_is_xyz){
      std::ifstream file(argv[filenames[0]]);
      if(!file.is_open()){
          std::cout << "Error: Could not find "<< argv[filenames[0]] << std::endl;
          return -1;
      }
      double x_,y_,z_;
      while(file >> x_ >> y_ >> z_){
          pcl::PointXYZRGB pt;
          pt.x = x_;
          pt.y = y_;
          pt.z= z_;
          cloud->points.push_back(pt);
      }

      pcl::console::print_info("\nFound txt file.\n");
      pcl::console::print_info ("[done, ");
      pcl::console::print_value ("%g", tt.toc ());
      pcl::console::print_info (" ms : ");
      pcl::console::print_value ("%d", cloud->size ());
      pcl::console::print_info (" points]\n");
  }

  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
  cloud->is_dense = true;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*cloud,*cloud_xyz);


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("MAP3D MESH"));


  viewer->setBackgroundColor(255, 255,255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud_xyz,0,0,0);
  viewer->addPointCloud(cloud_xyz,color_handler,"original_cloud");
  viewer->initCameraParameters ();
  viewer->resetCamera();

  std::string image = output_dir;
  image += "/screen_image.png";





  std::cout << image << std::endl;
  viewer->saveScreenshot(image.c_str());

  std::cout << "Press [q] to exit!" << std::endl;
  while (!viewer->wasStopped ()){
      viewer->spin();
  }



   return 0;
}

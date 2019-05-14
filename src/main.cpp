#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/common/common.h>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/gasd.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkPolyData.h>
#include <vtkCamera.h>
#include <vtkSphereSource.h>
#include <vtkLineSource.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkSmartPointer.h>
#include <vtkSimplePointsReader.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCaptionActor2D.h>
#include <vtkNamedColors.h>
#include <vtkTransform.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleUser.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkGraphicsFactory.h>
#include <vtkRegularPolygonSource.h>
#include <vtkCubeSource.h>
#include <vtkPLYWriter.h>
#include <vtkPLYReader.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>



void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose){
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
}

void vtkVisualizer(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,char ** argv){

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("VISUALIZER"));

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud,centroid);

  std::cout << "Centroid:" << centroid << std::endl;

  std::map<double,pcl::PointXYZ> points_sort;

  for(int i=0;i<cloud->points.size();i++){
      pcl::PointXYZ pt = cloud->points.at(i);
      double error = std::abs(pt.x-centroid[0]);
      points_sort[pt.x]=pt;
    }

  std::map<double,pcl::PointXYZ>::iterator it1 = points_sort.begin();
  pcl::PointXYZ min_inX = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(points_sort.end());
  pcl::PointXYZ max_inX = it2->second;

  std::cout << "min X:" << min_inX << std::endl;
  std::cout << "max X:" << max_inX << std::endl;

  double radius = std::abs(max_inX.x-centroid[0]);

  std::cout << "radius:" << radius << std::endl;


  viewer->setBackgroundColor(255,255,255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 0,0,0);
 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud, 0,255,0);
  viewer->addPointCloud(cloud,color, "cloud");

  //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,3,"DBH5m",PORT2);
  // Create a circle
  vtkSmartPointer<vtkRegularPolygonSource> polygonSource = vtkSmartPointer<vtkRegularPolygonSource>::New();

  //polygonSource->GeneratePolygonOff(); // Uncomment this line to generate only the outline of the circle
  polygonSource->GeneratePolygonOn();
  polygonSource->SetNumberOfSides(150);
  polygonSource->SetRadius(radius);
  polygonSource->SetCenter(centroid[0], centroid[1], centroid[2]);
  polygonSource->Update();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mesh (new pcl::PointCloud<pcl::PointXYZ>());
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  pts->DeepCopy(polygonSource->GetOutput()->GetPoints());

  vtkSmartPointer<vtkPolyData> cloudVTK = vtkSmartPointer<vtkPolyData>::New();
  cloudVTK->SetPoints(pts);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(cloudVTK);
  vertexFilter->Update();

  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  coefficients_cylinder->values.resize(3);
  coefficients_cylinder->values[0]=centroid[0];
  coefficients_cylinder->values[1]=centroid[1];
  coefficients_cylinder->values[3]=radius;

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(vertexFilter->GetOutput());

  std::map<double,pcl::PointXYZ> points_sort_forY;

  for(int i=0;i<cloud->points.size();i++){
     pcl::PointXYZ pt = cloud->points.at(i);
     double error = std::abs(pt.y-centroid[1]);
     points_sort_forY[pt.y]=pt;
  }

  std::map<double,pcl::PointXYZ>::iterator it3 = points_sort_forY.begin();
  pcl::PointXYZ min_inY = it3->second;

  std::map<double,pcl::PointXYZ>::iterator it4 = std::prev(points_sort_forY.end());
  pcl::PointXYZ max_inY = it4->second;

  std::cout << "min Y:" << min_inY << std::endl;
  std::cout << "max Y:" << max_inY << std::endl;

  pcl::io::vtkPolyDataToPointCloud(polydata,*cloud_mesh);
  //viewer->addPolygonMesh(polydata,"mesh1");
/*
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PassThrough<pcl::PointXYZ> pass1;
  pass1.setInputCloud(cloud_mesh);
  pass1.setFilterFieldName("y");
  pass1.setFilterLimits(-900,min_inY.y);
  pass1.setFilterLimitsNegative(true);
  pass1.filter(*cloud_filtered1);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>());

  pcl::PassThrough<pcl::PointXYZ> pass2;
  pass2.setInputCloud(cloud_filtered1);
  pass2.setFilterFieldName("y");
  pass2.setFilterLimits(max_inY.y, 3*max_inY.y);
  pass2.setFilterLimitsNegative(true);
  pass2.filter(*cloud_filtered2);
*/
  float min,max;
  //pass2.getFilterLimits(min,max);

  //std::cout << "Passthrough filter min:" << min << std::endl;
  //std::cout << "Passthrough filter max:" << max << std::endl;

  viewer->addPointCloud(cloud_mesh,color2,"circless");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,4,"circless");

  pcl::PointXYZ p1, p2, p3;
  p1.getArray3fMap() << 100, 0, 0;
  p2.getArray3fMap() << 0, 100, 0;
  p3.getArray3fMap() << 0,0.1,100;
/*
  viewer->addCoordinateSystem(100,"ucs");
  viewer->addText3D("x", p1, 15, 1, 0, 0, "x_");
  viewer->addText3D("y", p2, 15, 0, 1, 0, "y_");
  viewer->addText3D ("z", p3, 15, 0, 0, 1, "z_");
*/
  viewer->setPosition(0,0);
  viewer->initCameraParameters();

  std::string image = "screen_image.png";
  viewer->saveScreenshot(image.c_str());

  viewer->resetCamera();

  std::cout << "Press [q] to exit" << std::endl;

  while(!viewer->wasStopped ()) {
      viewer->spin();
    }

  cv::Mat img = cv::imread(image.c_str(),1);
/*
  canvas circleDetector;


  fitEllipseQ       = true;
  fitEllipseAMSQ    = false;
  fitEllipseDirectQ = false;
*/
  cv::Mat img_copy = img.clone();

  cv::Mat gray;

  cv::cvtColor(img_copy,gray,cv::COLOR_BGR2GRAY);
/*
  cv::namedWindow("gray",CV_WINDOW_NORMAL);
  cv::resizeWindow("gray",640,480);
  cv::moveWindow("gray",0,0);
  cv::imshow("gray",img_input);
  cv::waitKey(0);
  cv::destroyAllWindows();



  // Create toolbars. HighGUI use.
  createTrackbar( "threshold", "result", &sliderPos, 255, processImage );
  processImage(0, 0);
  // Wait for a key stroke; the same function arranges events processing
  waitKey(0);

  std::cout << "center final:" << box.center << std::endl;
  std::cout << "radius final:" << box.size.height << std::endl;
*/
  // circle outline
 // cv::circle(img_copy, box.center, box.size.width,cv::Scalar(255,255,0),5);




  std::vector<cv::Vec3f> circles(1);


  std::cout << "Filtering...GaussianBlur!" << std::endl;
  cv::GaussianBlur(gray,gray,cv::Size(3,3),0.2,0.2);

    std::cout << "\nFiltering...Canny!" << std::endl;
 // cv::Canny(gray, gray, 100, 100*2,3);
  cv::Canny(gray, gray, 84, 255);

  cv::namedWindow("gray",cv::WINDOW_NORMAL);
  cv::resizeWindow("gray",640,480);
  cv::moveWindow("gray",0,0);
  cv::imshow("gray",gray);
  cv::waitKey(0);
  cv::destroyAllWindows();

  int r = 0;
  cv::Point2d center_pattern1;
  int dp = 2;
  int minDist = 300;
  int param1 = 200;
  int param2 = 100;
  int minRadius = 0;
  int maxRadius = 0;



  while(true){

        cv::Mat temp = img.clone();

      cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT,dp,minDist, param1, param2, minRadius,maxRadius);
      for(size_t i = 0; i < circles.size(); i++ ){

          cv::Point2d center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          // center_pattern2 = center_pattern1;
          center_pattern1 = cv::Point2d(center.x,center.y);
          r = cvRound(circles[i][2]);
          // circle center
          cv::circle(temp, center, 1, cv::Scalar(255,0,0),5);
          // circle outline
          cv::circle(temp, center, r,cv::Scalar(255,0,0),5);

          break;
        }

      std::cout << "Center:"  << center_pattern1 << std::endl;
      std::cout << "Radius:"  << r << std::endl;

      //center_pattern1.x = 340;
      //center_pattern1.y = 216;
      //r = 94;

      //cv::circle(img_copy, center_pattern1, 1, cv::Scalar(255,0,0),5);
      // circle outline
      //cv::circle(img_copy, center_pattern1, r,cv::Scalar(255,0,0),5);

      cv::namedWindow("circle",cv::WINDOW_NORMAL);
      cv::resizeWindow("circle",640,480);
      cv::moveWindow("circle",0,0);
      cv::imshow("circle",temp);
      cv::waitKey(0);
      cv::destroyAllWindows();
/*
      int answer =-1;

      std::cout << "Correct? 1 --> yes, 2 --> no" << std::endl;

      std::cin >> answer;

      if(answer == 1){
          break;
        }else if(answer == 2){
          std::cout << "dp:" << std::endl;
          std::cin >> dp;
          std::cout << "minDist:" << std::endl;
          std::cin >> minDist;
          std::cout << "param1:" << std::endl;
          std::cin >> param1;
          std::cout << "param2:" << std::endl;
          std::cin >> param2;
          std::cout << "minRadius:" << std::endl;
          std::cin >> minRadius;
          std::cout << "maxRadius:" << std::endl;
          std::cin >> maxRadius;
          continue;
        }
*/
      break;

  }

  r = r*r;
  int inside_circle = 0;
  int total_circle=0;

  for(int i=0;i<img_copy.rows; i++){
      for(int j=0;j<img_copy.cols;j++){

          double d2 = std::pow(j-center_pattern1.x,2)+std::pow(i-center_pattern1.y,2);

          if(d2<r){

              total_circle += 1;

              cv::Vec3b color = img_copy.at<cv::Vec3b>(cv::Point(i,j));


              //std::cout << "color:" << color.val[0] << "," << color.val[1] << "," << color.val[2]<< std::endl;
              if(img_copy.at<cv::Vec3b>(i,j)[0]<100 or img_copy.at<cv::Vec3b>(i,j)[1]<100 or img_copy.at<cv::Vec3b>(i,j)[2]<100){
                  continue;
              }
              /*
              color[0] = 255;
              color[1] = 255;
              color[2] = 0;
              */
              //img_copy.at<cv::Vec3b>(cv::Point(i,j)) = color;

              img_copy.at<cv::Vec3b>(i,j)[0] = 0;
              img_copy.at<cv::Vec3b>(i,j)[1] = 255;
              img_copy.at<cv::Vec3b>(i,j)[2] = 0;

              inside_circle +=1;

            }else if(d2==r){continue;}
          else{
              continue;
            }


        }
    }

  std::cout << "Total pixel in circle:"  << total_circle << std::endl;
  std::cout << "Total differente crown pixel:" << inside_circle << std::endl;

  float resu = (float)inside_circle/total_circle;
  std::cout << "weight:" << resu << std::endl;
  int percentage = std::round(resu*100);

  std::cout << "Percentage canopy missing:" << percentage << "%" << std::endl;

  cv::namedWindow("fin",cv::WINDOW_NORMAL);
  cv::resizeWindow("fin",640,480);
  cv::moveWindow("fin",0,0);
  cv::imshow("fin",img_copy);
  cv::waitKey(0);
  cv::destroyAllWindows();
  

  std::string percenCanopyMissingFile = argv[2];
  percenCanopyMissingFile += "/percentageCanopyMissing.txt";

  ofstream out(percenCanopyMissingFile.c_str());
  out << "percentage: " << percentage << "%" << std::endl;
  out.close();

  //--------------------------------------
  // Cloud
  //--------------------------------------
  /*

  vtkSmartPointer<vtkPolyData> cloudVTK = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

  for(int n=0;n<cloud->points.size();n++){
    pcl::PointXYZ pt = cloud->points.at(n);
    pts->InsertNextPoint(pt.x,pt.y,pt.z);
  }

  cloudVTK->SetPoints(pts);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputData(cloudVTK);
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->ShallowCopy(vertexFilter->GetOutput());

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputData(polydata);

  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);
  actor1->GetProperty()->SetColor(0, 0,0);
  actor1->GetProperty()->SetPointSize(1);

  //--------------------------------------
  // Sphere
  //--------------------------------------

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud,centroid);

  std::cout << "Centroid:" << centroid << std::endl;

  std::map<double,pcl::PointXYZ> points_sort;

  for(int i=0;i<cloud->points.size();i++){
     pcl::PointXYZ pt = cloud->points.at(i);
     double error = std::abs(pt.x-centroid[0]);
     points_sort[pt.x]=pt;
  }

  std::map<double,pcl::PointXYZ>::iterator it1 = points_sort.begin();
  pcl::PointXYZ min_inX = it1->second;

  std::map<double,pcl::PointXYZ>::iterator it2 = std::prev(points_sort.end());
  pcl::PointXYZ max_inX = it2->second;

  double radius = std::abs(max_inX.x-centroid[0]);

  std::cout << "Radius:" << radius << std::endl;
*/
/*
  // Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(centroid[0], centroid[1], centroid[2]);
  sphereSource->SetRadius(radius);

  vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);
  */
  /*
  // Create a circle
   vtkSmartPointer<vtkRegularPolygonSource> polygonSource =
     vtkSmartPointer<vtkRegularPolygonSource>::New();

   //polygonSource->GeneratePolygonOff(); // Uncomment this line to generate only the outline of the circle
   polygonSource->SetNumberOfSides(50);
   polygonSource->SetRadius(radius);
   polygonSource->SetCenter(centroid[0], centroid[1], centroid[2]);

   std::map<double,pcl::PointXYZ> points_sort_forY;

   for(int i=0;i<cloud->points.size();i++){
      pcl::PointXYZ pt = cloud->points.at(i);
      double error = std::abs(pt.y-centroid[1]);
      points_sort_forY[pt.y]=pt;
   }

   std::map<double,pcl::PointXYZ>::iterator it3 = points_sort_forY.begin();
   pcl::PointXYZ min_inY = it3->second;

   std::map<double,pcl::PointXYZ>::iterator it4 = std::prev(points_sort_forY.end());
   pcl::PointXYZ max_inY = it4->second;

   // Create a cube.
     vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
     cubeSource->SetXLength(4*max_inX.x);
     cubeSource->SetYLength(radius);
     cubeSource->SetZLength(4*max_inX.x);
     cubeSource->SetCenter(centroid[0],min_inY.y,centroid[2]);


   // Visualize
   vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
   mapper2->SetInputConnection(polygonSource->GetOutputPort());

   vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
   actor2->SetMapper(mapper2);
   actor2->GetProperty()->SetColor(0,255,0);
   actor2->GetProperty()->SetPointSize(1);


   // Create a mapper and actor
   vtkSmartPointer<vtkPolyDataMapper> mapper3 = vtkSmartPointer<vtkPolyDataMapper>::New();
   mapper3->SetInputConnection(cubeSource->GetOutputPort());

   //mapper3->SetInputData(plane1);
   vtkSmartPointer<vtkActor> actor3 = vtkSmartPointer<vtkActor>::New();
   actor3->SetMapper(mapper3);
   actor3->GetProperty()->SetColor(255,255,255);

  //----------------------------------------------
  //----------------------------------------------

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  //renderer->SetBackground(0.0, 0.0, 0.0);
  // Zoom in a little by accessing the camera and invoking its "Zoom" method.
  //renderer->ResetCamera();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();

  renderWindow->SetSize(800, 600);
  renderWindow->AddRenderer(renderer);
  //renderWindow->SetOffScreenRendering(1);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Screenshot
 vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();

  // Add the actor to the scene
  //renderer->AddActor(axes);
   renderer->SetBackground(255,255,255);
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);
  renderer->AddActor(actor3);
  //renderer->AddActor(actor4);


  renderer->ResetCamera();
  renderer->SetViewPoint(0,0,0);

  renderWindow->Render();
  // renderer->ResetCamera();


  // Render and interact

  renderWindow->SetWindowName("VTK VISUALIZER");

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);

  windowToImageFilter->SetInput(renderWindow);
 // windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
//  windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
  windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName("screenshot2.png");
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();

  vtkSmartPointer<vtkPLYWriter> plyWriter = vtkSmartPointer<vtkPLYWriter>::New();
   plyWriter->SetFileName("cloud");
   plyWriter->SetInputConnection(polygonSource->GetOutputPort());
   plyWriter->Write();



  renderWindowInteractor->Start();
  */
}

int main(int argc, char** argv){

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  
  if(argc <3 or argc > 3){
  

      std::cout << "\nEnter: " << argv[0] << " <cloud.pcd> <output dir>"  << std::endl <<
                   "[q] to exit" << std::endl;
      return (-1);
  
  }

  if(pcl::io::loadPCDFile<pcl::PointXYZ> (argv[1], *cloud) == -1){
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
  }

  std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from pcd with the following fields: "
              << std::endl;

  vtkVisualizer(cloud,argv);


  /*
pcl::PointCloud<PointType>::Ptr cylinderPoints
pcl::PCA<PointType> pca;
pcl::PointCloud<PointType> proj;

pca.setInputCloud (cloud);
pca.project (*cloud, proj);

PointType proj_min;
PointType proj_max;
pcl::getMinMax3D (proj, proj_min, proj_max);
   */

  return 0;
}


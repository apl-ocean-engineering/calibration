// Blatantly ``borrowed'' from  PCL visualization demo
//
/* \author Geoffrey Biggs */


#include <iostream>
#include <fstream>
#include <string>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <glog/logging.h>
#include <tclap/CmdLine.h>

#include "sonar_pose.h"
#include "distortion_model.h"
#include "camera_factory.h"

using namespace std;
using namespace cv;
using namespace Distortion;


class VisualizerOpts {
public:
  VisualizerOpts( void )
  {;}

  string pcFile, imageOverlay, cameraFile, cameraSonarFile, annotatedImage;
  bool imageAxes, doVisualize;

  bool parseCmdLine( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Visualizers", ' ', "0.1" );

      TCLAP::ValueArg< string > overlayImageArg("", "image-overlay", "Image to overlay", false, "", "Image to overlay", cmd );
      TCLAP::ValueArg< string > cameraFileArg("", "camera-calibration", "Camera calibration", false, "", "Calibration file", cmd );
      TCLAP::ValueArg< string > cameraSonarFileArg("", "camera-sonar", "Camera-sonar calibration", false, "", "Calibration file", cmd );
      TCLAP::ValueArg< string > annotatedImageArg("", "annotated-image", "Annotated image", false, "", "Image file", cmd );

      TCLAP::SwitchArg imgAxesArg( "", "image-axes", "Image axes", cmd, false );
      TCLAP::SwitchArg dontVisualizeArg( "", "dont-visualize", "Do visualize", cmd, false );

      TCLAP::UnlabeledValueArg< string > pcFileArg( "pc-file", "Point cloudfile", true, "", "File name", cmd );

      cmd.parse( argc, argv );

      pcFile = pcFileArg.getValue();

      imageOverlay = overlayImageArg.getValue();
      cameraFile   = cameraFileArg.getValue();
      cameraSonarFile = cameraSonarFileArg.getValue();
      annotatedImage = annotatedImageArg.getValue();
      doVisualize = (dontVisualizeArg.getValue() == false);

      imageAxes = imgAxesArg.getValue();


    } catch (TCLAP::ArgException &e) {
      LOG(ERROR) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate();
  }

  bool  validate( void )
  {
    bool overlay = imageOverlay.length() > 0,
    cam     = cameraFile.length() > 0,
    camson  = cameraSonarFile.length() > 0;

    if( overlay == false && cam == false && camson == false ) {
    } else if( overlay == true && cam == true && camson == true ) {
    } else {
      LOG(ERROR) << "All three image-overlay, camera-calibration and camera-sonar must be specified if any one is specified";
      return false;
    }


    return true;
  }

  bool doAnnotate( void )
  {
    return annotatedImage.length() > 0 && imageOverlay.length() > 0;
  }

};


class ColorModel {
public:
  virtual uint32_t color( const float x, const float y, const float z ) = 0;

  // A lot janky
  virtual vector< Vec2i > imagePoints( void ) const { return vector<Vec2i>(); }
};

class ConstantColor : public ColorModel {
public:
  ConstantColor( int r, int g, int b )
  : _r(r), _g(g), _b(b), _rgb( ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b) )
  {;}

  virtual uint32_t color( const float x, const float y, const float z )
  {
    return _rgb;
  }

protected:
  int _r, _g, _b;
  uint32_t _rgb;
};

class ImageOverlay : public ColorModel {
public:
  ImageOverlay( const Mat &img, DistortionModel *cam, SonarPose *pose )
  : _img( img ), _cam( cam ), _pose( pose ), _imgPts(), _haveOutput(0)
  {;}

  ~ImageOverlay()
  {
    delete _cam;
    delete _pose;
  }

  virtual uint32_t color( const float x, const float y, const float z )
  {
    // Transform point to image frame
    Vec3f inCamFrame( _pose->sonarToImage( Vec3f( x, y, z ) ) );
    Vec2d inCamDist( _cam->distort( inCamFrame ) );
    //Vec2d inImgDist( _cam->image( Vec2f(inImgFrame[0]/inImgFrame[2], inImgFrame[1]/inImgFrame[2] ) ) );
    Vec2f inImg( _cam->image(inCamDist) );

    int r,g,b;

    Vec2i intImg( round(inImg[0]), round(inImg[1]) );

    if( intImg[0] < 0 || intImg[0] > _img.size().width ||
    intImg[1] < 0 || intImg[1] > _img.size().height ) {
      r = g = b = 100;
    } else {

      if( ++_haveOutput < 10 ) {
          LOG(INFO) << Vec3f(x,y,z) << " -> " << inCamFrame << " -> " << inCamDist << " -> " << inImg;
      }

      Vec3b p( _img.at< Vec3b >( intImg[1], intImg[0] ) );

      _imgPts.push_back( inImg );

      //LOG(INFO) << p;

      r = p[0];
      g = p[1];
      b = p[2];
    }


    return ( ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b) );
  }

  static ImageOverlay *Construct( const string &imgFile, const string &camFile, const string &camSonFile )
  {
    DistortionModel *camera = CameraFactory::LoadDistortionModel( camFile );
    Mat img( imread( imgFile ) );
    SonarPose *pose = SonarPose::Load( camSonFile );

    return new ImageOverlay( img, camera, pose );
  }

  virtual vector< Vec2i > imagePoints( void ) const { return _imgPts; }

protected:

  Mat _img;
  DistortionModel *_cam;
  SonarPose *_pose;
  vector< Vec2i > _imgPts;

  int _haveOutput;

};


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;


  VisualizerOpts opts;
  if( !opts.parseCmdLine( argc, argv ) ) exit(-1);

  ColorModel *model;
  if( opts.imageOverlay.length() > 0 ) {
    LOG(INFO) << "Constructing ImageOverlay";
    model = ImageOverlay::Construct( opts.imageOverlay, opts.cameraFile, opts.cameraSonarFile );
  } else {
    model = new ConstantColor( 150, 150, 150 );
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr basic_cloud_ptr( new  pcl::PointCloud<pcl::PointXYZRGB> );

  // Load XYZ file

  ifstream infile( opts.pcFile );

  if( !infile.is_open() ) {
    LOG(ERROR) << "Error opening point cloud file \"" << opts.pcFile << "\"";
    exit(-1);
  }

  while( !infile.eof() ) {
    float x,y,z,r;

    infile >> x >> y >> z >> r;

    pcl::PointXYZRGB point;
    point.x = x;

    if( opts.imageAxes ) {
      point.y = -z;
      point.z = y;
    } else {
      point.y = y;
      point.z = z;
    }

    uint32_t rgb = model->color( point.x, point.y, point.z );
    point.rgb = *reinterpret_cast<float*>(&rgb);

    basic_cloud_ptr->points.push_back( point );

  }

  infile.close();

  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;

  if( opts.doAnnotate() ) {
    Mat img = imread( opts.imageOverlay );
    vector< Vec2i > pts = model->imagePoints();

    LOG(INFO) << "Drawing annoated image with " << pts.size() << " points";
    for( int i = 0 ; i < pts.size(); ++i ) {
      circle( img, Point2i( pts[i][0], pts[i][1] ), 3, Scalar( 0,0,255 ), -1 );
    }

    imwrite( opts.annotatedImage, img );
  }


  //  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  //  std::cout << "Genarating example point clouds.\n\n";
  //  // We're going to make an ellipse extruded along the z-axis. The colour for
  //  // the XYZRGB cloud will gradually go from red to green to blue.
  //  uint8_t r(255), g(15), b(15);
  //  for (float z(-1.0); z <= 1.0; z += 0.05)
  //  {
  //    for (float angle(0.0); angle <= 360.0; angle += 5.0)
  //    {
  //      pcl::PointXYZ basic_point;
  //      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
  //      basic_point.y = sinf (pcl::deg2rad(angle));
  //      basic_point.z = z;
  //      basic_cloud_ptr->points.push_back(basic_point);
  //
  //      pcl::PointXYZRGB point;
  //      point.x = basic_point.x;
  //      point.y = basic_point.y;
  //      point.z = basic_point.z;
  //      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
  //          static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
  //      point.rgb = *reinterpret_cast<float*>(&rgb);
  //      point_cloud_ptr->points.push_back (point);
  //    }
  //    if (z < 0.0)
  //    {
  //      r -= 12;
  //      g += 12;
  //    }
  //    else
  //    {
  //      g -= 12;
  //      b += 12;
  //    }
  //  }
  //  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  //  basic_cloud_ptr->height = 1;
  //  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
  //  point_cloud_ptr->height = 1;
  //
  //  // ----------------------------------------------------------------
  //  // -----Calculate surface normals with a search radius of 0.05-----
  //  // ----------------------------------------------------------------
  //  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  //  ne.setInputCloud (point_cloud_ptr);
  //  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  //  ne.setSearchMethod (tree);
  //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  //  ne.setRadiusSearch (0.05);
  //  ne.compute (*cloud_normals1);
  //
  //  // ---------------------------------------------------------------
  //  // -----Calculate surface normals with a search radius of 0.1-----
  //  // ---------------------------------------------------------------
  //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  //  ne.setRadiusSearch (0.1);
  //  ne.compute (*cloud_normals2);

  if( opts.doVisualize ) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(basic_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> ( basic_cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
  }

  return 0;
}



#ifdef FALSE

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
  {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
  }


  boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //------------------------------------
    //-----Add shapes at cloud points-----
    //------------------------------------
    viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
      cloud->points[cloud->size() - 1], "line");
      viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

      //---------------------------------------
      //-----Add shapes at other locations-----
      //---------------------------------------
      pcl::ModelCoefficients coeffs;
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      viewer->addPlane (coeffs, "plane");
      coeffs.values.clear ();
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (5.0);
      viewer->addCone (coeffs, "cone");

      return (viewer);
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
      {
        // --------------------------------------------------------
        // -----Open 3D viewer and add point cloud and normals-----
        // --------------------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
        viewer->addCoordinateSystem (1.0);

        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

        return (viewer);
      }


      unsigned int text_id = 0;
      void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
        void* viewer_void)
        {
          boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
          if (event.getKeySym () == "r" && event.keyDown ())
          {
            std::cout << "r was pressed => removing all text" << std::endl;

            char str[512];
            for (unsigned int i = 0; i < text_id; ++i)
            {
              sprintf (str, "text#%03d", i);
              viewer->removeShape (str);
            }
            text_id = 0;
          }
        }

        void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
          void* viewer_void)
          {
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
            if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
            event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
            {
              std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

              char str[512];
              sprintf (str, "text#%03d", text_id ++);
              viewer->addText ("clicked here", event.getX (), event.getY (), str);
            }
          }

          boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
          {
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);
            viewer->addCoordinateSystem (1.0);

            viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
            viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

            return (viewer);
          }

          #endif

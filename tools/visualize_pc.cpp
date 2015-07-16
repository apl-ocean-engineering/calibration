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

#include "sonar_image_warper.h"
#include "sonar_detections.h"

#include "background_segmenter.h"

using namespace std;
using namespace cv;
using namespace Distortion;


class VisualizerOpts {
public:
  VisualizerOpts( void )
  : annotateMode( NONE )
  {;}

  string pcFile, imageOverlay, cameraCalibration, cameraSonarFile, annotatedImage;
  string sonarFile, cameraFile, backgroundFile;
  bool imageAxes, doDisplay, dropNonImaged;

  enum AnnotateMode { NONE = -1, OVERLAY, SEGMENT } annotateMode;

  bool parseCmdLine( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("Visualizers", ' ', "0.1" );

      TCLAP::ValueArg< string > cameraFileArg("", "camera-detections", "Camera detection file", false, "", "Detections file", cmd );
      TCLAP::ValueArg< string > sonarFileArg("", "sonar-detections", "sonar detection file", false, "", "Detections file", cmd );
      TCLAP::ValueArg< string > overlayImageArg("", "image-overlay", "Image to overlay", false, "", "Image to overlay", cmd );
TCLAP::ValueArg< string > backgroundImageArg("", "background-image", "Image for background", false, "", "Image for background", cmd );
      TCLAP::ValueArg< string > cameraCalArg("", "camera-calibration", "Camera calibration", false, "", "Calibration file", cmd );
      TCLAP::ValueArg< string > cameraSonarFileArg("", "camera-sonar", "Camera-sonar calibration", false, "", "Calibration file", cmd );

      TCLAP::ValueArg< string > annotatedArg("", "annotate", "Annotation mode", false, "", "{}", cmd );
      TCLAP::ValueArg< string > annotatedImageArg("", "annotated-image", "Annotated image", false, "", "Image file", cmd );

      TCLAP::SwitchArg imgAxesArg( "", "use-image-axes", "Image axes", cmd, false );
      TCLAP::SwitchArg dropNonImagedArg( "", "drop-non-imaged", "", cmd, false );
      TCLAP::SwitchArg doDisplayArg( "", "do-display", "Do display", cmd, false );

      TCLAP::UnlabeledValueArg< string > pcFileArg( "pc-file", "Point cloudfile", true, "", "File name", cmd );

      cmd.parse( argc, argv );

      pcFile = pcFileArg.getValue();

      imageOverlay = overlayImageArg.getValue();
      cameraFile   = cameraFileArg.getValue();
      sonarFile    = sonarFileArg.getValue();
      cameraCalibration = cameraCalArg.getValue();
      cameraSonarFile = cameraSonarFileArg.getValue();
      annotatedImage = annotatedImageArg.getValue();
backgroundFile = backgroundImageArg.getValue();
      doDisplay = doDisplayArg.getValue();
      dropNonImaged = dropNonImagedArg.getValue();

      if( annotatedArg.isSet() ) {
        string arg( annotatedArg.getValue() );
        if( arg.compare( "overlay") == 0 )      annotateMode = OVERLAY;
        else if( arg.compare("segment") == 0 )  annotateMode = SEGMENT;
        else {
          LOG(ERROR) << "Couldn't understant annotation mode \"" << arg << "\"";
          return false;
        }
      }

      imageAxes = imgAxesArg.getValue();


    } catch (TCLAP::ArgException &e) {
      LOG(ERROR) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate();
  }

  bool  validate( void )
  {

    bool overlay = imageOverlay.length() > 0,
    cam     = cameraCalibration.length() > 0,
    camson  = cameraSonarFile.length() > 0;

    if( overlay == false && cam == false && camson == false ) {
    } else if( overlay == true && cam == true && camson == true ) {
    } else {
      LOG(ERROR) << "All three image-overlay, camera-calibration and camera-sonar must be specified if any one is specified";
      return false;
    }

    if( annotateMode != NONE ) {
      // Validate for annotation modes
    } else if( annotatedImage.length() > 0) {
      LOG(ERROR) << "Annotated image specified, but annotate mode not given.";
      return false;
    }

    return true;
  }

  bool doAnnotate( void )
  {
    return annotateMode != NONE;
  }

};


class ColorModel {
public:
  virtual ~ColorModel() {;}

  virtual bool color( const float x, const float y, const float z, uint32_t &rgb ) = 0;

  // A lot janky
  virtual vector< Vec2i > imagePoints( void ) const { return vector<Vec2i>(); }
};

class ConstantColor : public ColorModel {
public:
  ConstantColor( int r, int g, int b )
  : _r(r), _g(g), _b(b), _rgb( ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b) )
  {;}

  virtual bool color( const float x, const float y, const float z, uint32_t &rgb )
  {
    rgb = _rgb;
    return true;
  }

protected:
  int _r, _g, _b;
  uint32_t _rgb;
};

class ImageOverlay : public ColorModel {
public:
  ImageOverlay( const Mat &img, SonarImageWarper *warper )
  : _img( img ), _warper( warper ), _imgPts(), _haveOutput(0)
  {;}

  ~ImageOverlay()
  {  }

  virtual bool color( const float x, const float y, const float z, uint32_t &rgb )
  {
    Vec2f inImg( _warper->sonarToImage( x,y,z) );
    int r,g,b;
    bool visible = false;

    Vec2i intImg( round(inImg[0]), round(inImg[1]) );

  //   if( ++_haveOutput < 10 ) {
  // LOG(INFO) << Vec3f(x,y,z) << " -> " << inImg;
  //   }

    if( intImg[0] < 0 || intImg[0] >= _img.size().width ||
    intImg[1] < 0 || intImg[1] >= _img.size().height ) {
      r = g = b = 100;
    } else {

      Vec3b p( _img.at< Vec3b >( intImg[1], intImg[0] ) );

      _imgPts.push_back( inImg );

      r = p[0];
      g = p[1];
      b = p[2];
      visible = true;
    }


    rgb = ( ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b) );
    return visible;
  }


  virtual vector< Vec2i > imagePoints( void ) const { return _imgPts; }

protected:

  Mat _img;
  SonarImageWarper *_warper;
  vector< Vec2i > _imgPts;

  int _haveOutput;

};

class PCVisualizer {
public:

  PCVisualizer( VisualizerOpts &opts_ )
  : opts(opts_), model( NULL ), warper( NULL ), cloud_ptr( NULL )
  {;}

  ~PCVisualizer()
  {
    if( model != NULL ) delete model;
    if( warper != NULL ) delete warper;
  }

  int run( void )
  {

    if( opts.imageOverlay.length() > 0 ) {
      LOG(INFO) << "Constructing ImageOverlay";
      warper = new SonarImageWarper( opts.cameraCalibration, opts.cameraSonarFile );
      model = new ImageOverlay( imread( opts.imageOverlay ), warper );
    } else {
      model = new ConstantColor( 150, 150, 150 );
    }

    if( model == NULL ) {
      LOG(ERROR) << "Fatal error, color model not created.";
      return -1;
    }

    //  This should satisfy boost::shared_ptr
    cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

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

      uint32_t rgb;
      bool visible = model->color( point.x, point.y, point.z, rgb );

      if( visible or (opts.dropNonImaged == false) ) {
        point.rgb = *reinterpret_cast<float*>(&rgb);
        cloud_ptr->points.push_back( point );
      }

    }

    infile.close();

    cloud_ptr->width = (int) cloud_ptr->points.size ();
    cloud_ptr->height = 1;

    if( opts.doAnnotate() ) doAnnotate(  );
    else doVisualize();



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




    return 0;


  }

  int doAnnotate( void )
  {
    Mat out;

    switch( opts.annotateMode ) {
      case VisualizerOpts::OVERLAY:
      out = annotateOverlay();
      break;
      case VisualizerOpts::SEGMENT:
      out =  annotateSegment();
      break;
      case VisualizerOpts::NONE:
      default:
      LOG(INFO) << "Hm, no annotation mode selected.";
    }

    if( opts.annotatedImage.length() > 0 ) {
      imwrite( opts.annotatedImage, out );
      LOG(INFO) << "Wrote annotated image to " << opts.annotatedImage;
    }

    if( opts.doDisplay ) {
      imshow("visualize_pc", out );

      char c = waitKey(0);
    }

    return 0;
  }

  Mat annotateOverlay( void ) {
    Mat img = imread( opts.imageOverlay );
    vector< Vec2i > pts = model->imagePoints();

    LOG(INFO) << "Drawing annotated image with " << pts.size() << " points";
    for( int i = 0 ; i < pts.size(); ++i ) {
      circle( img, Point2i( pts[i][0], pts[i][1] ), 3, Scalar( 0,0,255 ), -1 );
    }

    if( (opts.sonarFile.length() > 0) and (warper != NULL) ) {
      LOG(INFO) << "Drawing sonar sphere";

      SonarDetections dets;
      dets.load( opts.sonarFile, opts.imageAxes );
      SonarDetection *det = dets.find( timestamp() );

      if( det != NULL ) {
        ImageDetection *imageDet = det->projectToImage( warper );

        imageDet->draw( img );
        delete imageDet;

      } else {
        LOG(INFO) << "Couldn't find timestamp \"" << timestamp() << "\" in sonar file " << opts.sonarFile;
      }
    }

    return img;
  }

  Mat annotateSegment( void )
  {
    Mat overlay = imread( opts.imageOverlay ), mask( Mat::zeros(overlay.size(), CV_8UC1 ) );

    _bgSeg.setImage( overlay );

    if( opts.backgroundFile.length() > 0 )  {
      LOG(INFO) << "Loading background image \"" << opts.backgroundFile << "\"";
      LOG(INFO) << "Performing background segmentation";
      _bgSeg.loadBackground( opts.backgroundFile );
    }

    vector< Vec2i > pts = model->imagePoints();

    LOG(INFO) << "Drawing annotated image with " << pts.size() << " points";
    for( int i = 0 ; i < pts.size(); ++i ) {
      Point2i imagePoint( pts[i][0], pts[i][1] );
      // Find the points in the PointCloud

      //if( _bgSeg.isForeground( imagePoint ) ) {
        // pcl::PointXYZRGB pc( cloud_ptr->points[i] );
        // float dist = sqrt( pc.x*pc.x + pc.y*pc.y + pc.z+pc.z );
        //
        // // Half a degree in Rad
        // float radius = dist * sin(0.008726646259971648);
        // LOG(INFO) << "Dist = " << dist << " ; radius = " << radius;

        float radius = 3.0;

        circle( mask, imagePoint, radius, Scalar( 255 ), -1 );
      //}
    }

    Mat out( Mat::zeros(overlay.size(), overlay.type() ) );
    overlay.copyTo( out, mask );

    return out;
  }


  int doVisualize( void )
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> ( cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.25, 0, 0, 0 );
    viewer->initCameraParameters ();

    // If imageAxes, attempt to align camera so it's still "upright"
    // that is, initial view, +X is to right, +Y is down, +Z is away
    if( opts.imageAxes ) viewer->setCameraPosition( 0, 0, 0, 0, -1, 0 );

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      //
      // char c = getchar();
      // switch(c) {
      //   case 'q': viewer->close(); break;
      //
      // }
    }

    return 0;
  }

  string timestamp()
  {
    if( opts.imageOverlay.length() > 0 ) {
      return boost::filesystem::path( opts.imageOverlay).stem().string();
    } else {
      return string("");
    }
  }

protected:

  VisualizerOpts &opts;
BackgroundSegmenter _bgSeg;
  ColorModel *model;
  SonarImageWarper *warper;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
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

  PCVisualizer viz( opts );
  return viz.run();
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

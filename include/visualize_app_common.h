// Blatantly ``borrowed'' from  PCL visualization demo
//
/* \author Geoffrey Biggs */

#ifndef __VISUALIZE_APP_COMMON_H__
#define __VISUALIZE_APP_COMMON_H__


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

  {;}

  string pcFile, imageOverlay, cameraCalibration, cameraSonarFile;
  string sonarFile, cameraFile, backgroundFile;
  bool imageAxes, doDisplay, dropNonImaged;

  enum AnnotateMode { NONE = -1, OVERLAY, SEGMENT } annotateMode;

  int _argc;
  char **_argv;

  bool parseCmdLine( int argc, char **argv )
  {
    _argc = argc;
    _argv = argv;

    try {
      TCLAP::CmdLine cmd("Visualizers", ' ', "0.1" );
      doParseCmdLine( cmd );
    } catch (TCLAP::ArgException &e) {
      LOG(ERROR) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate();
  }

  virtual void doParseCmdLine( TCLAP::CmdLine &cmd )
  {

    TCLAP::ValueArg< string > cameraFileArg("", "camera-detections", "Camera detection file", false, "", "Detections file", cmd );
    TCLAP::ValueArg< string > sonarFileArg("", "sonar-detections", "sonar detection file", false, "", "Detections file", cmd );
    TCLAP::ValueArg< string > overlayImageArg("", "image-overlay", "Image to overlay", false, "", "Image to overlay", cmd );
    TCLAP::ValueArg< string > backgroundImageArg("", "background-image", "Image for background", false, "", "Image for background", cmd );
    TCLAP::ValueArg< string > cameraCalArg("", "camera-calibration", "Camera calibration", false, "", "Calibration file", cmd );
    TCLAP::ValueArg< string > cameraSonarFileArg("", "camera-sonar", "Camera-sonar calibration", false, "", "Calibration file", cmd );

    TCLAP::SwitchArg imgAxesArg( "", "use-image-axes", "Image axes", cmd, false );
    TCLAP::SwitchArg dropNonImagedArg( "", "drop-non-imaged", "", cmd, false );
    TCLAP::SwitchArg doDisplayArg( "", "do-display", "Do display", cmd, false );

    TCLAP::UnlabeledValueArg< string > pcFileArg( "pc-file", "Point cloudfile", true, "", "File name", cmd );

    cmd.parse( _argc, _argv );

    pcFile = pcFileArg.getValue();

    imageOverlay = overlayImageArg.getValue();
    cameraFile   = cameraFileArg.getValue();
    sonarFile    = sonarFileArg.getValue();
    cameraCalibration = cameraCalArg.getValue();
    cameraSonarFile = cameraSonarFileArg.getValue();
    backgroundFile = backgroundImageArg.getValue();
    doDisplay = doDisplayArg.getValue();
    dropNonImaged = dropNonImagedArg.getValue();

    imageAxes = imgAxesArg.getValue();
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

  return true;
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

class VisualizerCommon {
public:

  VisualizerCommon( VisualizerOpts &opts_ )
  : opts(opts_), model( NULL ), warper( NULL ), cloud_ptr( NULL )
  {;}

  virtual ~VisualizerCommon()
  {
    if( model != NULL ) delete model;
    if( warper != NULL ) delete warper;
  }

  virtual int run( void ) = 0;

  int loadModels( void )
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

#endif

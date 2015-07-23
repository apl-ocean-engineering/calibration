

#include <glog/logging.h>


#include "visualize_app_common.h"

using namespace std;
using namespace cv;
using namespace Distortion;


class ImgVisualizerOpts : public VisualizerOpts {
public:
  ImgVisualizerOpts( void )
  : annotateMode( NONE )
  {;}

  string annotatedImage;
  enum AnnotateMode { NONE = -1, OVERLAY, SEGMENT } annotateMode;

  virtual void doParseCmdLine( TCLAP::CmdLine &cmd )
  {

    TCLAP::ValueArg< string > annotatedArg("", "annotate", "Annotation mode", true, "", "{}", cmd );
    TCLAP::ValueArg< string > annotatedImageArg("", "annotated-image", "Annotated image", false, "", "Image file", cmd );

    VisualizerOpts::doParseCmdLine( cmd );

    annotatedImage = annotatedImageArg.getValue();

    if( annotatedArg.isSet() ) {
      string arg( annotatedArg.getValue() );
      if( arg.compare( "overlay") == 0 )      annotateMode = OVERLAY;
      else if( arg.compare("segment") == 0 )  annotateMode = SEGMENT;
      else {
        LOG(ERROR) << "Couldn't understant annotation mode \"" << arg << "\"";
        return;
      }
    }
  }

  virtual bool validate( void )
  {

    if( annotateMode != NONE ) {
      // Validate for annotation modes
    } else if( annotatedImage.length() > 0) {
      LOG(ERROR) << "Annotated image specified, but annotate mode not given.";
      return false;
    }

    return VisualizerOpts::validate();
  }

};



class ImgVisualizer : public VisualizerCommon {
public:

  ImgVisualizer( ImgVisualizerOpts &opts_ )
  : VisualizerCommon( opts_ ), opts(opts_)
  {;}

  virtual ~ImgVisualizer()
  {
  }

  virtual int run( void )
  {
    VisualizerCommon::loadModels();

    doAnnotate(  );

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

      float imgRadius = 10;

      if( warper ) {
        //pcl::PointXYZRGB pc( cloud_ptr->points[i] );
        //float dist = sqrt( pc.x*pc.x + pc.y*pc.y + pc.z+pc.z );
        //
        // Half a degree in Rad
        //float radius = dist * sin(0.008726646259971648);
        float radius = tan(0.008726646259971648);
        imgRadius = radius * warper->cam()->favg();

        //LOG(INFO) << "Dist = " << dist << " ; radius = " << radius << " ; imgRadius " << imgRadius;

      }
      //float radius = 3.0;

      circle( mask, imagePoint, imgRadius, Scalar( 255 ), -1 );
      //}
    }

    Mat out( Mat::zeros(overlay.size(), overlay.type() ) );
    overlay.copyTo( out, mask );

    return out;
  }

protected:

  ImgVisualizerOpts &opts;


};



// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  ImgVisualizerOpts opts;
  if( !opts.parseCmdLine( argc, argv ) ) exit(-1);

  ImgVisualizer viz( opts );
  return viz.run();
}

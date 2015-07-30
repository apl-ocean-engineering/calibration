

#include <glog/logging.h>

#include "graphcut.h"

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
  bool refineSegmentation;
  enum AnnotateMode { NONE = -1, OVERLAY, SEGMENT } annotateMode;

  virtual void doParseCmdLine( TCLAP::CmdLine &cmd )
  {

    TCLAP::ValueArg< string > annotatedArg("", "annotate", "Annotation mode", true, "", "{}", cmd );
    TCLAP::ValueArg< string > annotatedImageArg("", "annotated-image", "Annotated image", false, "", "Image file", cmd );

    TCLAP::SwitchArg doRefineSegmentationArg( "", "refine-segmentation", "Refine segmentations", cmd, false );

    VisualizerOpts::doParseCmdLine( cmd );

    annotatedImage = annotatedImageArg.getValue();
    refineSegmentation = doRefineSegmentationArg.getValue();

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
    Mat out( Mat::zeros(overlay.size(), overlay.type() ) );

    _bgSeg.setImage( overlay );

    if( opts.backgroundFile.length() > 0 )  {
      LOG(INFO) << "Loading background image \"" << opts.backgroundFile << "\"";
      LOG(INFO) << "Performing background segmentation";
      _bgSeg.loadBackground( opts.backgroundFile );
    }

    if( warper ) {

      if( opts.showOutliers ) {
        LOG(INFO) << "Drawing annotated image with " << outliers->size() << " outlier points";
        for( int i = 0; i < outliers->size(); ++i ) {
          const Scalar outlierColor( 20, 20, 20 );

          pcl::PointXYZRGB pt( outliers->points[i] );
          cv::Point imagePoint( warper->sonarToImage( pt.x, pt.y, pt.z ) );
          circle( out, imagePoint, 3, outlierColor, -1 );
        }
      }

      LOG(INFO) << "Drawing annotated image with " << inliers->size() << " inlier points";
      for( int i = 0 ; i < inliers->size(); ++i ) {
        //if( _bgSeg.isForeground( imagePoint ) ) {

        pcl::PointXYZRGB pt( inliers->points[i] );
        cv::Point imagePoint( warper->sonarToImage( pt.x, pt.y, pt.z ) );

        //float dist = sqrt( pc.x*pc.x + pc.y*pc.y + pc.z+pc.z );
        //
        // Half a degree in Rad
        //float radius = dist * sin(0.008726646259971648);
        float radius = tan(0.008726646259971648);
        float imgRadius = radius * warper->cam()->favg();

        //LOG(INFO) << "Dist = " << dist << " ; radius = " << radius << " ; imgRadius " << imgRadius;

        circle( mask, imagePoint, imgRadius, Scalar( 255 ), -1 );
        //}
      }
    }

    Mat refinedMask;
    if( opts.refineSegmentation ) {
      LOG(INFO) << "Refining segmentation.";
      grabCutRefinement( overlay, mask, refinedMask );
      //activeContoursRefinement( overlay, mask, refinedMask );
      overlay.copyTo( out, refinedMask );
    } else {
      overlay.copyTo( out, mask );
    }

    return out;
  }

  void activeContoursRefinement( const Mat &img, const Mat &mask, Mat &out )
  {
    int niters = 6;

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    Mat temp;
    dilate(mask, temp, Mat(), Point(-1,-1), niters);
    erode(temp, temp, Mat(), Point(-1,-1), niters*2);
    dilate(temp, temp, Mat(), Point(-1,-1), niters);

    imshow( "mask", temp );

    findContours( temp, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE );

    int count = 0;
    for( int i = 0; i < contours.size(); ++i ) count += contours[i].size();

    LOG(INFO) << "Extracted of " << contours.size() << " contours, totalling " << count << " points";

    Mat dst;
    img.copyTo( dst, temp );

    // Draw contours
    for( int i = 0; i < contours.size(); ++i )
    {
      vector<Point> &pts( contours[i] );
      Scalar color( 0, 0, 255 );
      const int thickness = 4;

      LOG(INFO) << "Contour " << i << " has " << pts.size() << " points";

      for( int j = 0; j < (pts.size()-1); ++j ) line( dst, pts[i], pts[i+1], color, thickness );

      if( pts.size() > 2 ) line( dst, pts[ pts.size() - 1 ], pts[0], color, thickness );
    }

    imshow( "activeContours", dst);
    waitKey(0);

    out = mask;
  }

  void grabCutRefinement( const Mat &img, const Mat &mask, Mat &out )
  {
    //const int numLabels = 2;
    const int numIter = 30;
    const string SegmentationWindow("refineSegmentation");

    // Create a number of masks.
    //   Background = !( dilated many times )
    //   Foreground = eroded many times
    //   Prob foreground = dilated - foreground
    //   Prob background = background - prob foreground

    // Hmm, well you have to classify everything, eh?
    const int FGErodeIterations = 20;
    Mat defFG;
    erode( mask, defFG, Mat(), Point(-1,-1), FGErodeIterations );

    const int probFGDilateIterations = 20;
    Mat probFG;
    dilate( mask, probFG, Mat(), Point(-1,-1), probFGDilateIterations );
    imshow( "mask", mask );

    const int probBGDilateIterations = 40;
    Mat probBG;
    dilate( probFG, probBG, Mat(), Point(-1,-1), probBGDilateIterations );
    //     Mat probBGInv( img.size(), CV_8UC1, Scalar( 1 ) );
    // probBGInv.setTo( Scalar( 0 ), probBG );

    //     const int defBGDilateIterations = 50;
    //     Mat defBG;
    //     dilate( probBG, defBG, Mat(), Point(-1,-1), defBGDilateIterations );
    //     Mat defBGInv( img.size(), CV_8UC1, Scalar( 1 ) );
    // defBGInv.setTo( Scalar( 0 ), defBG );

    // construct initial mask, order matters
    Mat grabCutMask( img.size(), CV_8UC1, Scalar( GC_PR_BGD ) );
    //grabCutMask.setTo( GC_PR_BGD, probBG );
    grabCutMask.setTo( GC_PR_FGD, probFG );
    //grabCutMask.setTo( GC_FGD, defFG );

    Mat gcMaskImage;
    drawGrabCutMask( grabCutMask, gcMaskImage );
    imshow( SegmentationWindow, gcMaskImage );

    Mat bgModel, fgModel;

    for( int i = 0; i < numIter; ++i ) {
      LOG(INFO) << "Performing GrabCut iter " << i;

      int gcMode = ( i == 0 ? GC_INIT_WITH_MASK : GC_EVAL );
      graphCut( img, grabCutMask, Rect(), bgModel, fgModel, 1, gcMode );

      drawGrabCutMask( grabCutMask, gcMaskImage );
      imshow( SegmentationWindow, gcMaskImage );

      // If we weren't displaying the image, this could go outside the loop.
      Mat binMask( img.size(), CV_8UC1, Scalar(0));
      binMask = grabCutMask & GC_FGD;
      out = binMask;

      Mat maskedImage;
      img.copyTo( maskedImage, binMask );
      imshow( "refinedImage", maskedImage );
      waitKey(1);
    }


  }

  void drawGrabCutMask( const Mat &mask, Mat &image )
  {
    image = Mat::zeros( mask.size(), CV_8UC3 );

    const Scalar Red( 0, 0, 255 ), Yellow(0, 255, 255),
    Green(0, 255, 0), Blue(255,0,0), Black(0,0,0);

    // Tried to do this with masks and bitwise operations.
    // image.setTo( Red, mask & GC_FGD );
    // image.setTo( Yellow, mask & GC_PR_FGD );
    // image.setTo( Green, mask & GC_PR_BGD );
    // image.setTo( Blue, mask & GC_BGD );

    // The GC_* flags are bitwise, so that didn't work.
    // could do something like
    Mat bitmask;
    cv::compare( mask, Scalar( GC_FGD ), bitmask, CMP_EQ );
    image.setTo( Red, bitmask );

    cv::compare( mask, Scalar( GC_PR_FGD ), bitmask, CMP_EQ );
    image.setTo( Yellow, bitmask );

    cv::compare( mask, Scalar( GC_PR_BGD ), bitmask, CMP_EQ );
    image.setTo( Green, bitmask );

    cv::compare( mask, Scalar( GC_BGD ), bitmask, CMP_EQ );
    image.setTo( Blue, bitmask );

    // ....

    // Size sz( image.size() );
    // for( unsigned int i = 0; i < sz.height; ++i ) {
    //   for( unsigned int j = 0; j < sz.width; ++j ) {
    //     Scalar color;
    //     switch( mask.at<unsigned char>(i,j) ) {
    //       case GC_FGD:    color = Red; break;
    //       case GC_PR_FGD: color = Yellow; break;
    //       case GC_PR_BGD: color = Green; break;
    //       case GC_BGD:    color = Blue; break;
    //       default:        color = Black;
    //     }
    //     image.at< Scalar >(i,j) = color;
    //   }
    // }

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

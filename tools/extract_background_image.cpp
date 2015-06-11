
#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "image_accumulator.h"
#include "composite_canvas.h"

using namespace std;
using namespace cv;

using namespace AplCam;

namespace fs = boost::filesystem;

struct ExtractBgOpts {
 public:
  ExtractBgOpts()
  {;}

  string inFile, outputImg, outputMat;

  bool doDisplay;

  int startSeconds, duration, stereoImage;


  bool parseOpts( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("deinterlace", ' ', "0.1" );

      TCLAP::UnlabeledValueArg< std::string > inFileArg( "input-file", "Video file", true, "", "a filename", cmd );
      TCLAP::ValueArg< std::string > outputImgArg("o", "output-image", "Output image", false, "", "a filename", cmd );
      TCLAP::ValueArg< std::string > outputMatArg("m", "output-mat", "Output mat", false, "", "a filename", cmd );


      TCLAP::SwitchArg doDisplayArg("X", "do-display", "Display video while processing", cmd, false );

      TCLAP::ValueArg< float > startSecondsArg( "S", "start-seconds", "Seconds to start to", false, 0.0, "seconds", cmd );
      TCLAP::ValueArg< float > durationArg( "d", "duration", "Number of seconds of video to extract", false, 1.0, "seconds", cmd );
      TCLAP::ValueArg< int > stereoImageArg( "", "stereo-image", "Stereo image", false, -1, "0 or 1", cmd );

      cmd.parse( argc, argv );

      // Extract arguments from TCLAP;
      inFile = inFileArg.getValue(); 

      outputImg = outputImgArg.getValue();
      outputMat = outputMatArg.getValue();

      doDisplay = doDisplayArg.getValue();

      startSeconds = startSecondsArg.getValue();
      duration     = durationArg.getValue();

      stereoImage  = stereoImageArg.getValue();

    } catch( TCLAP::ArgException &e ) {
      LOG(ERROR) << "error: " << e.error() << " for arg " << e.argId();
      return false;
    }

    return validate();
  }

  bool validate( void )
  {
    return true;
  }

  //int seekTo( float fps )
  //{
  //  cout << fps << " " << seekFrame << " " << seekSeconds << endl;
  //  // exclusive nature of these two flags is handled in validate()
  //  if( seekFrame > 0 )
  //    return seekFrame;
  //  else if( seekSeconds > 0 ) {
  //    return round(seekSeconds * fps);
  //  } else
  //    return -1;
  //}

};




class ExtractBgMain {
 public:
  ExtractBgMain( ExtractBgOpts &options )
      : opts( options )
  {;}



  int run( void ) {

    if( opts.stereoImage  < 0 )
      return doSingle();
    else
      return doComposite();

  }

  // Too tired to fix the DRY right now
  int doSingle( void )
  {
    VideoCapture vid( opts.inFile );
    if( !vid.isOpened() ) {
      cout << "Error opening video" << endl;
      return -1;
    }

    float fps = vid.get( CV_CAP_PROP_FPS );

    int frame = opts.startSeconds * fps;

    if( frame > 0 ) {
      LOG(INFO) << "Jumping to frame " << frame << "(" << opts.startSeconds << " seconds)" << endl;
      vid.set( CV_CAP_PROP_POS_FRAMES, frame );
    }

    int durationFrame = opts.duration * fps;

    Mat img;
    int count = 0;
    while( vid.read( img ) and count++ < durationFrame ) {
      processImage( img );
    }

    doOutput( img );
    return 0;
  }


  int doComposite( void ) {

    CompositeVideo vid( opts.inFile );
    if( !vid.isOpened() ) {
      cout << "Error opening video" << endl;
      return -1;
    }

    float fps = vid.fps();

    int frame = opts.startSeconds * fps;

    if( frame > 0 ) {
      LOG(INFO) << "Jumping to frame " << frame << "(" << opts.startSeconds << " seconds)" << endl;
      vid.seek( frame );
    }

    int durationFrame = opts.duration * fps;

    CompositeCanvas canvas;
    int count = 0;
    while( vid.read( canvas ) and count++ < durationFrame ) {
      processImage( canvas[ opts.stereoImage ] );
    }

    doOutput( canvas[ opts.stereoImage] );
    return 0;
  }

  void processImage( const Mat &img )
  {
      accum.add( img );

      if( opts.doDisplay ) {

        unsigned char ch = 0;

        imshow( vidWindowName, img );

        Mat meanImg;
        accum.mean().convertTo( meanImg, img.type() );
        imshow( meanWindowName, meanImg );

        char c = waitKey(1);

      }
  }


  void doOutput( const Mat &img )
  {

    if( accum.size() > 0 ) {
      if( opts.outputImg.length() > 0 ) {
        Mat meanImg;
        accum.mean().convertTo( meanImg, img.type());

        imwrite( opts.outputImg, meanImg );
      }

      if( opts.outputMat.length() > 0 ) {

        FileStorage fs( opts.outputMat, FileStorage::WRITE );

        fs << "mean" << accum.mean();
        fs << "var"  << accum.var();
      }

    }



  }



 private:

  ExtractBgOpts opts;
    ImageAccumulator accum;

  const string vidWindowName = "Input image",
        meanWindowName = "Mean image";
};



int main( int argc, char **argv ) 
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  ExtractBgOpts opts;
  if( !opts.parseOpts( argc, argv ) )  exit(-1);

  ExtractBgMain main( opts );

  exit( main.run() );
}


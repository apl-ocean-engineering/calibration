
#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>

#include <tclap/CmdLine.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "composite_canvas.h"

using namespace std;
using namespace cv;

using namespace AplCam;

namespace fs = boost::filesystem;

struct DeinterlaceOpts {
  public:
    DeinterlaceOpts()
      : inFile(""), 
      doDisplay( false ),
      waitKey( 1 ), 
      seekFrame( -1 ), 
      seekSeconds( -1.0 )
  {;}

    string inFile;
    bool doDisplay;
    int waitKey, seekFrame;
    float seekSeconds;

    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream strm;

      try {
        TCLAP::CmdLine cmd("deinterlace", ' ', "0.1" );

        TCLAP::UnlabeledValueArg< std::string > inFileArg( "input-file", "File to deinterlace", true, "", "a filename", cmd );

        TCLAP::SwitchArg doDisplayArg( "D", "do-display", "Display video while processing", cmd );

        TCLAP::ValueArg< int > waitKeyArg( "W", "wait-key", "Minimum interval between images if displaying", false, 1, "interval in ms", cmd );

        TCLAP::ValueArg< int > seekFrameArg( "s", "seek", "Frame to seek to", false, -1, "frame", cmd );
        TCLAP::ValueArg< float > seekSecondsArg( "S", "seek-seconds", "Seconds to seek to", false, -1.0, "seconds", cmd );

        cmd.parse( argc, argv );

        // Extract arguments from TCLAP;
        inFile = inFileArg.getValue(); 
        doDisplay = doDisplayArg.getValue();
        waitKey = waitKeyArg.getValue();

        seekFrame = seekFrameArg.getValue();
        seekSeconds = seekSecondsArg.getValue();

      } catch( TCLAP::ArgException &e ) {
        strm << "error: " << e.error() << " for arg " << e.argId();
        msg = strm.str();
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      if( seekFrame > 0 && seekSeconds > 0 ) {
        msg = "Cannot set both seek seconds and seek frame.";
        return false;
      }

      return true;
    }

    int seekTo( float fps )
    {
      cout << fps << " " << seekFrame << " " << seekSeconds << endl;
      // exclusive nature of these two flags is handled in validate()
      if( seekFrame > 0 )
        return seekFrame;
      else if( seekSeconds > 0 ) {
        cout << seekSeconds * fps <<  " " << round(seekSeconds * fps) << endl;
        return round(seekSeconds * fps);
      } else
        return -1;
    }

};




class DeinterlaceMain
{
  public:
    DeinterlaceMain( DeinterlaceOpts &options )
      : opts( options )
    {;}


    int run( void ) {

      cout << "Deinterlacing " << opts.inFile << endl;

      VideoCapture vid( opts.inFile );
      if( !vid.isOpened() ) {
        cout << "Error opening video" << endl;
        return -1;
      }

      int vidWidth = vid.get( CV_CAP_PROP_FRAME_WIDTH ),
          vidHeight = vid.get( CV_CAP_PROP_FRAME_HEIGHT );
      Size compositeSize( vidWidth, vidHeight/2 );
      VerticalCompositeCanvas canvas( compositeSize, CV_8UC3 );

      if( opts.doDisplay ) namedWindow( "deinterlace" );
      int wait = opts.waitKey;

      int frame = opts.seekTo( vid.get( CV_CAP_PROP_FPS ) );
      if( frame > 0 ) {
        cout << "Jumping to frame " << frame << endl;
        vid.set( CV_CAP_PROP_POS_FRAMES, frame );
      }


      Mat img;
      bool doStop = false;
      while( vid.read( img ) and doStop == false) {
        // Try doing this by direct manipulation

        Mat lines[2] = { Mat( img ), Mat( img ) };

        lines[1].data += lines[1].step;

        for( int i = 0; i < 2; ++i ) {
          cout << lines[i].step[0] << " " << lines[i].step[1] << endl;
          lines[i].step[0] *= 2;
          lines[i].size[0] /= 2;
          lines[i].flags &= ~Mat::CONTINUOUS_FLAG;
        }

        assert( (lines[0].rows + lines[1].rows) == img.rows );

        if( opts.doDisplay ) {

          for( int i = 0; i < 2; ++i ) lines[i].copyTo( canvas[i] );

          imshow( "deinterlace", canvas );

          unsigned char ch = waitKey( wait );
          switch( ch ) {
            case 'q': doStop = true; break;
            case ' ': 
                      if( wait == 0 ) wait = opts.waitKey;
                      else wait = 0;
                      break;
          }
        }

      }
      return 0;
    }



  private:
    DeinterlaceOpts opts;
};



int main( int argc, char **argv ) 
{

  DeinterlaceOpts opts;
  string msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg << endl;
    exit(-1);
  }

  DeinterlaceMain main( opts );

  exit( main.run() );

}


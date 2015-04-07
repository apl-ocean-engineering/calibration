
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
      waitKey( 1 )
  {;}

    string inFile;
    bool doDisplay;
    int waitKey;

    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream strm;

      try {
        TCLAP::CmdLine cmd("deinterlace", ' ', "0.1" );

        TCLAP::UnlabeledValueArg< std::string > inFileArg( "input-file", "File to deinterlace", true, "", "a filename", cmd );

        TCLAP::SwitchArg doDisplayArg( "D", "do-display", "Display video while processing", cmd );

        TCLAP::ValueArg< int > waitKeyArg( "W", "wait-key", "Minimum interval between images if displaying", false, 1, "interval in ms", cmd );



        cmd.parse( argc, argv );

        // Extract arguments from TCLAP;
        inFile = inFileArg.getValue(); 
        doDisplay = doDisplayArg.getValue();
        waitKey = waitKeyArg.getValue();


      } catch( TCLAP::ArgException &e ) {
        strm << "error: " << e.error() << " for arg " << e.argId();
        msg = strm.str();
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      return true;
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
      Size compositeSize( vidWidth*2, vidHeight/2 );
      CompositeCanvas canvas( compositeSize, CV_8UC3 );

      if( opts.doDisplay ) namedWindow( "deinterlace" );

      Mat img;
      while( vid.read( img ) ) {
        // Try doing this by direct manipulation

        Mat lines[2] = { Mat( img ), Mat( img ) };

        lines[1].data += lines[1].step;

        for( int i = 0; i < 2; ++i ) {
          lines[i].step[0] *= 2;
          lines[i].rows /= 2;
        }

        assert( (lines[0].rows + lines[1].rows) == img.rows );

        if( opts.doDisplay ) {

          for( int i = 0; i < 2; ++i ) lines[i].copyTo( canvas[i] );

          imshow( "deinterlace", canvas );

          unsigned char ch = waitKey( opts.waitKey );
          switch( ch ) {
            case 'q': break;
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


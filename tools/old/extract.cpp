
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "trendnet_time_code.h"

using namespace std;
using namespace cv;

namespace fs = boost::filesystem;

enum Mode { PRETRIGGER, ARMED, WAIT };

struct ExtractOpts {
  public:
    ExtractOpts()
      : seekTo(-1), intervalFrames(-1), waitKey( 1 ),
      intervalSeconds( -1 ),
      dataDir("data"),
      boardName(),
      doDisplay( false ), yes( false ),
      verb( NONE )
  {;}


    typedef enum {EXTRACT_SINGLE, EXTRACT_INTERVAL,  NONE = -1} Verbs;

    int seekTo, intervalFrames, waitKey;
    float intervalSeconds;
    string dataDir;
    string boardName;
    bool doDisplay, yes;
    Verbs verb;
    vector< string > inFiles;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    //== Option parsing and help ==

    string help( void )
    {
      stringstream strm;

      strm <<  "This is a tool for extracting images from a video file." << endl;
      //    "Usage: calibration\n"
      //    "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
      //    "     --board,-b <board_name>    # Name of calibration pattern\n"
      //    "     --camera, -c <camera_name> # Name of camera\n"
      //    "     --seek-to <frame>          # Seek to specified frame before starting\n"
      //    //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
      //    //     "                              # (used only for video capturing)\n"
      //    //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
      //    //     "     [-op]                    # write detected feature points\n"
      //    //     "     [-oe]                    # write extrinsic parameters\n"
      //    //     "     [-zt]                    # assume zero tangential distortion\n"
      //    //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
      //    //     "     [-p]                     # fix the principal point at the center\n"
      //    //     "     [-v]                     # flip the captured images around the horizontal axis\n"
      //    //     "     [-V]                     # use a video file, and not an image list, uses\n"
      //    //     "                              # [input_data] string for the video file name\n"
      //    //     "     [-su]                    # show undistorted images after calibration\n"
      //    "     [input_data]             # list of files to use\n"
      //    "\n" );
      //printf("\n%s",usage);
      //printf( "\n%s", liveCaptureHelp );

      return strm.str();
    }

    string unknownOption( int opt )
    {
      stringstream strm;
      strm << "Unknown option \"" << opt << "\"";
      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      static struct option long_options[] = {
        { "data_directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "seek-to", true, NULL, 's' },
        { "interval-frames", true, NULL, 'i' },
        { "interval-seconds", true, NULL, 'I' },
        { "do-display", no_argument,  NULL, 'x' },
        { "yes", no_argument, NULL, 'y' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      if( argc < 2 )
      {
        msg =  help();
        return false;
      }

      int indexPtr;
      int optVal;
      while( (optVal = getopt_long( argc, argv, "b:c:d:i:s:x?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'd':
            dataDir = optarg;
            break;
          case 'b':
            boardName = optarg;
            break;
          case 'i':
            intervalFrames = atoi( optarg );
            break;
          case 'I':
            intervalSeconds = atof( optarg );
            break;
          case 's':
            seekTo = atoi( optarg );
            break;
          case 'x':
            doDisplay = true;
            break;
          case 'y':
            yes = true;
            break;
          case '?':
            msg = help();
            return false;
            break;
          default:
            msg = unknownOption( optopt );
            return false;
        }
      }

      if( optind == argc )
      {
        msg = help();
        return false;
      }

      string verbIn = argv[optind++];
      for( int i = optind; i < argc; ++i ) inFiles.push_back( argv[i] );

      if( verbIn.compare("single") == 0 ) {
        verb = EXTRACT_SINGLE;

        if( inFiles.size() < 2 ) {
          msg = "Need to specify a video file and an image file name on the command line.\n"
            "    usage:   extract single <video_file> <output_image>";

          return false;
        }
      } else if( verbIn.compare("interval") == 0 ) {
verb = EXTRACT_INTERVAL;

        if( inFiles.size() < 2 ) {
          msg = "Need to specify a video file and a destination directory on the command line.\n"
            "    usage:   extract single <video_file> <dest_directory>";

          return false;
        }

        if( intervalFrames > 0 && intervalSeconds > 0 ) {
          msg = "Can't specify both interval frames and interval seconds at the same time.";
          return false;
        }
      } else {
        msg = "Don't understand \"" + verbIn +  "\"";
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      return true;
    }

};




class ExtractMain
{
  public:
    ExtractMain( ExtractOpts &options )
      : opts( options )
    {;}


    int run( void ) {
      if( opts.doDisplay ) namedWindow( "Extract" );

      switch( opts.verb ) {
        case ExtractOpts::EXTRACT_SINGLE:
          return doExtractSingle();
          case ExtractOpts::EXTRACT_INTERVAL:
return doExtractInterval();
        default:
          return -1;
      }

      return -1;
    }

    int doExtractSingle( void )
    {
      string videoSource( opts.inFiles[0] );
      string imageOut( opts.inFiles[1] );

      VideoCapture vid( videoSource );

      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }

      if( opts.seekTo > 0 ) {
        int videoLength = vid.get( CV_CAP_PROP_FRAME_COUNT );
        if( opts.seekTo > videoLength ) {
          cerr << "Requested seek-to of " << opts.seekTo << " after end of video " << videoLength << endl;
          return -1;
        }

        vid.set( CV_CAP_PROP_POS_FRAMES, opts.seekTo );
      }

      Mat img;
      vid.read( img );

      cout << "Wrote to \"" << imageOut << "\"" << endl;
      imwrite( imageOut.c_str(), img );

      return 0;
    }

    int doExtractInterval( void )
    {
      string videoSource( opts.inFiles[0] );
      fs::path directoryOut( opts.inFiles[1] );

      VideoCapture vid( videoSource );

      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }

      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );
      int digits = ceil( log10( vidLength ) );

      if( opts.intervalSeconds > 0 ) opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );

      if( (opts.intervalFrames > 0) && (vidLength / opts.intervalFrames > 1000) ) {
        cerr << "This interval will result in " << vidLength / opts.intervalFrames << "images being extracted.  Please confirm you want this with the '--yes' option." << endl;
        return -1;
      }


      if( !fs::is_directory( directoryOut ) ) fs::create_directories( directoryOut );

      Mat img;
      while( vid.read( img ) ) {
        char filename[40];
        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
        snprintf( filename, 39, "frame_%0*d.png", digits, currentFrame );
        fs::path fullPath( directoryOut );
        fullPath /= filename;

        cout << "Output frame " << currentFrame << " to " << fullPath.string() << endl;
        imwrite( fullPath.c_str(), img );

        if( opts.doDisplay ) {
          imshow("Extract",img );
          waitKey( opts.waitKey );
        }


        if( opts.intervalFrames > 1 ) {
          int destFrame = currentFrame + opts.intervalFrames - 1;
          if( destFrame < vidLength ) {
            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
          } else {
            // Done
            break;
          }
        }
      }

      return 0;


    }



  private:
    ExtractOpts opts;
};



int main( int argc, char **argv )
{

  ExtractOpts opts;
  string msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg << endl;
    exit(-1);
  }

  ExtractMain main( opts );

  exit( main.run() );

}


#ifdef CRUFT
int opt;
string path("/tmp/watch"), prefix("save");

while( (opt = getopt( argc, argv, "p:d:")) != -1  ) {
  switch( opt ) {
    case 'p':
      prefix = optarg;
      break;
    case 'd':
      path = optarg;
      break;
    case '?':
      cout << "Don't recognize option \"" << optopt << "\'"<< endl;
      exit(-1);
  }
}


if( optind >= argc ) {
  cout << "Usage: extract_frame <filename>" << endl;
  exit(0);
}




// Currently static, should be read from file (?)
const float consecutiveSec = 0.25, standoffSec = 1;
enum Mode state = PRETRIGGER;

string videofile( argv[optind] );

cout << "Opening " << videofile << endl;
VideoCapture capture;
if( isnumber(videofile.c_str()[0]) )
  capture.open( atoi( videofile.c_str() ) );
  else
  capture.open( videofile.c_str() );


  double fps = capture.get( CV_CAP_PROP_FPS );
  if( fps == 0 ) fps = 30;
  const int consecutive = ceil( consecutiveSec * fps ),
  standoff = ceil( standoffSec * fps );

cout << "Waits for " << consecutive << " consecutive frames" << endl;

if( !capture.isOpened() ) {
  cout << "Could not initialize video capture for " << videofile << endl;
  exit(-1);
}

Mat img, grey, prev, scaled;
Mat prevTimeCode;
int frame = 0, imgCount = 0, counter = 0;
bool flash = false;

while( capture.read( img ) ) {
  ++frame;
  cvtColor( img, grey, CV_BGR2GRAY );

  Mat imgTC( grey, timeCodeROI_1920x1080 ), imgTimeCode;
  imgTC.setTo( 0 );

  Mat shrunk;
  resize( grey, shrunk, Size(), 0.25, 0.25, INTER_AREA );
  flip( shrunk, scaled, 1 );

  if( !prev.empty() ) {

    double norm = cv::norm( scaled, prev, NORM_L2 ); // * scaled.size().area();

    cout << frame << " : " << state << "," << counter << " : " <<
      std::setw(10) << std::setprecision(1) << std::fixed << norm << endl;

    // Implement a kind of hysteresis..
    int motionLevel = (state == ARMED ? 1500 : 1500);
    if( norm > motionLevel ) {
      // Motion

      switch( state ) {
        case PRETRIGGER:
          state = ARMED;
          counter = 0;
          break;
        case ARMED:
          counter = 0;
          break;
        case WAIT:
          if( ++counter > standoff ) {
            state = ARMED;
            counter = 0;
          }
          break;
      }

    } else {
      // No motion

      switch( state ) {
        case PRETRIGGER:
          break;
        case ARMED:
          if( ++counter > consecutive ) {
            char timecode[40], filename[256];
            time_t tt;
            time( &tt );
            strftime( timecode, 39, "%y%m%d_%H%M%S", localtime( &tt ) );
            snprintf( filename, 255, "%s/%s_%s.png", path.c_str(), prefix.c_str(), timecode );
            cout << "Save to " << filename << endl;
            imwrite( filename, img );
            flash = true;

            state = WAIT;
            counter = 0;
          }
          break;
        case WAIT:
          break;
      }


    }


  }

  imshow("diff", scaled-prev );

  // Must be a deep copy or prev never changes
  scaled.copyTo( prev );

  // Put this after assignment above .. lets me invert in place when I "flash"
  if( flash ) bitwise_not( scaled, scaled );
  imshow("Video", scaled );
  waitKey( flash ? 1000 : 1 );
  flash = false;


}


#endif

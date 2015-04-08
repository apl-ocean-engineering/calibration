
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#ifdef USE_TBB
#include "tbb/tbb.h"
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "trendnet_time_code.h"
using namespace AplCam;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

struct BuildDbOpts {
  public:
    BuildDbOpts()
      : seekTo(-1), intervalFrames(-1), waitKey( 1 ), 
      intervalSeconds( -1 ),
      dataDir("data"),
      boardName(), 
      benchmarkFile(),
      doBenchmark( false ),
      doRewrite( false ),
      doDisplay( false ), yes( false ),
      verb( NONE )
  {;}


    typedef enum {  NONE = -1} Verbs;

    int seekTo, intervalFrames, waitKey;
    float intervalSeconds;
    string dataDir;
    string boardName, benchmarkFile;
    bool doBenchmark, doRewrite, doDisplay, yes;
    Verbs verb;

    string inFile;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cachePath( const string &file = "" ) const
    { return dataDir + "/cache/" + file; }

    //== Option parsing and help ==

    string help( void )
    {
      stringstream strm;
      const int w = 20;

      strm <<  "This is a tool for extracting images from a video file." << endl;
      strm << setw( w ) <<  "--data-directory, -d" << "Set location of data directory." << endl;
      strm << setw( w ) << "--do-display, -x" << "Do display video" << endl;
      strm << setw( w ) << "--do-benchmark [file]" << "Save benchmarking information to [file]" << endl;
      strm << setw( w ) << "--do-rewrite" << "Extract features even if it already exists in database" << endl;
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

    string unknownOption( const char opt )
    {
      stringstream strm;
      strm << "Unknown option \"" << opt << "\"";
      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "board", true, NULL, 'b' },
        { "seek-to", true, NULL, 's' },
        { "interval-frames", true, NULL, 'i' },
        { "interval-seconds", true, NULL, 'I' },
        { "do-rewrite", no_argument, NULL, 'R' },
        { "do-display", no_argument,  NULL, 'x' },
        { "do-benchmark", required_argument, NULL, 'K' },
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
      while( (optVal = getopt_long( argc, argv, "b:c:d:K:I:i:Rs:x?", long_options, &indexPtr )) != -1 ) {
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
          case 'K':
            doBenchmark = true;
            benchmarkFile = optarg;
            break;
          case 'R':
            doRewrite = true;
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
            msg = unknownOption( optVal );
            return false;
        }
      }

      if( optind == argc )
      {
        msg = help();
        return false;
      }

      inFile = argv[optind];

      if( intervalFrames > 0 && intervalSeconds > 0 ) {
        msg = "Can't specify both interval frames and interval seconds at the same time.";
        return false;
      }

      return validate( msg );
    }

    bool validate( string &msg )
    {
      return true;
    }

};




class BuildDbMain 
{
  public:
    BuildDbMain( BuildDbOpts &options )
      : opts( options ), _benchmark() 
    {;}

    ~BuildDbMain( void )
    {
      if( _benchmark.is_open() ) _benchmark.close();
    }

    int run( void ) {
      if( opts.doDisplay ) namedWindow( "BuildDb" );

      return doBuildDb();
    }

    int doBuildDb( void )
    {
      board = Board::load( opts.boardPath(), opts.boardName );
      if( !board ) {
        cerr << "Couldn't open board from " << opts.boardPath() << endl;
        return -1;
      }

      string videoSource( opts.inFile );
      VideoCapture vid( videoSource );
      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }

      mkdir_p( opts.cachePath() );
      if( ! db.open( opts.cachePath(), opts.inFile, true ) ) {
        cerr << "Error opening database file: " << db.error().name() << endl;
        return -1;
      }

      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

      if( opts.intervalSeconds > 0 ) 
        opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );

      FrameVec_t frames;
      const int chunkSize = 50;
      Mat img;

      while( vid.read( img ) ) {
        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
        cout << currentFrame << " ";

        if( !db.has( currentFrame ) || opts.doRewrite ) {
          frames.push_back( Frame( currentFrame, img.clone() ) );
        }

        if( opts.intervalFrames > 1 ) {
          int destFrame = currentFrame + opts.intervalFrames - 1;

          if( destFrame < vidLength ) 
            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
          else 
            break;
        }

        if( frames.size() >= chunkSize ) {
          cout << endl;
          processFrames( frames );
        }
      }

      cout << endl;
      processFrames( frames );


      return 0;
    }

    struct Frame {
      Frame( int _f, Mat _m )
        : frame(_f), img( _m )
      {;}

      int frame;
      Mat img;
    };

    typedef vector< Frame > FrameVec_t;
    typedef pair< int, int64 > TimingData_t;
    typedef vector< TimingData_t > TimingDataVec_t;

    void processFrames( FrameVec_t &frames )
    {
      AprilTagDetectorFunctor f( frames, db, board );

#ifdef USE_TBB
      tbb::parallel_reduce( tbb::blocked_range<size_t>(0, frames.size()), f );
#else
      f();
#endif

      cout << "Processed " <<  frames.size() << " frames,  produced " << f.timingData.size() <<  " timing data" << endl;

      if( opts.doBenchmark ) saveTimingData( f.timingData );

      frames.clear();
    }


    struct AprilTagDetectorFunctor {
      public:
        AprilTagDetectorFunctor( FrameVec_t &frames, DetectionDb &db, Board *board )
          : _frames( frames ), _db(db), _board( *board), timingData()
        {;}

        FrameVec_t &_frames;
        DetectionDb &_db;
        Board &_board;
        TimingDataVec_t timingData;

#ifdef USE_TBB
        // Splitting constructor for TBB
        AprilTagDetectorFunctor( AprilTagDetectorFunctor &other, tbb::split )
          : _frames( other._frames ), _db( other._db ), _board( other._board ), timingData()
        {;}


        void operator()( const tbb::blocked_range<size_t> &r ) 
        {

          size_t end = r.end();
          for( size_t i = r.begin(); i != end; ++i ) {
#else
            void operator()( ) 
            {

              size_t end = _frames.size();
              for( size_t i = 0; i < end; ++i ) {
#endif
                Detection *detection = NULL;
                Frame &p( _frames[i]);

                //cout << "Extracting from " << p.frame << ". ";

                Mat grey;
                cvtColor( p.img, grey, CV_BGR2GRAY );
                vector<Point2f> pointbuf;

                int64 before = getTickCount();
                detection = _board.detectPattern( grey, pointbuf );
                int64 elapsed = getTickCount() - before;

                //cout << p.frame << ": " << detection->size() << " features" << endl;

                // Trust the thread-safety of kyotocabinet
                _db.save( p.frame, *detection);

                int sz = detection->size();
                delete detection;

                timingData.push_back( make_pair( sz, elapsed ) );
              }
            }

#ifdef USE_TBB
            void join( const AprilTagDetectorFunctor &other )
            {
              std::copy( other.timingData.begin(), other.timingData.end(), back_inserter( timingData ) );
            }
#endif

          };


          void saveTimingData( const TimingDataVec_t &td )
          {
            if( !_benchmark.is_open() ) _benchmark.open( opts.benchmarkFile, ios_base::trunc );

            for( TimingDataVec_t::const_iterator itr = td.begin();  itr != td.end(); ++itr )
              _benchmark << (*itr).first << ',' << ((*itr).second / getTickFrequency()) << endl;
          }



          private:
          BuildDbOpts opts;

          DetectionDb db;
          Board *board;

          ofstream _benchmark;
        };



        int main( int argc, char **argv ) 
        {

          BuildDbOpts opts;
          string msg;
          if( !opts.parseOpts( argc, argv, msg ) ) {
            cout << msg << endl;
            exit(-1);
          }

          BuildDbMain main( opts );

          exit( main.run() );

        }



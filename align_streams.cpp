
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

#include <ffts.h>


#include "file_utils.h"
#include "trendnet_time_code.h"

using namespace cv;
using namespace std;

#define MAKE_NORMFILE
#ifdef MAKE_NORMFILE
ofstream normFile;
#endif

struct AlignmentOptions
{
  AlignmentOptions( void )
    : window( 5.0 ), maxDelta( 5.0 )
  {;}


  float window, maxDelta;
  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "max-delay", true, NULL, 'd'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:d:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case '?':
          help( msg );
          return false;
          break;
        default:
          stringstream strm;
          strm << "Unknown option \'" << optopt << "\'";
          msg = strm.str();
          return false;
          break;
      }

    }

    if( (argc - optind) < 2 ) {
      msg = "Must specify two video files on command line";
      return false;
    }

    video1 = argv[optind++];
    video2 = argv[optind];

    return validate( msg );
  }

  bool validate( string &msg )
  {
    if( !file_exists( video1 ) ) {
      msg = "File \'" + video1 + "\' doesn't exist";
      return false;
    }
    if( !file_exists( video2 ) ) {
      msg = "File \'" + video2 + "\' doesn't exist";
      return false;
    }

    return true;
  }

  void help( string &msg )
  {
    stringstream strm;
    strm << "Help!" << endl;

    msg = strm.str();
  }
};


struct TimecodeTransition
{
  TimecodeTransition( const Mat &b, const Mat &a )
    : before( b.clone() ), after( a.clone() )
  {;}

  Mat before, after;

};

class Video
{
  public:

    typedef map< int, TimecodeTransition > TransitionMap;

    Video( const string &file )
      : capture( file.c_str() ),filename( file )
    {
      // Should be more flexible about this..
      assert( (height() == 1080) && (width() == 1920) );
    }

    string filename;
    VideoCapture capture;

    float fps( void ) { return capture.get( CV_CAP_PROP_FPS ); }
    int frameCount( void ) { return capture.get( CV_CAP_PROP_FRAME_COUNT ); }
    int height( void ) { return capture.get( CV_CAP_PROP_FRAME_HEIGHT ); }
    int width( void ) { return capture.get( CV_CAP_PROP_FRAME_WIDTH ); }

    string dump( void ) 
    {
      stringstream strm;

      strm << "File " << filename << ": ";
      strm << width() << " x " << height() << ", ";
      strm << fps() << " fps.";

      return strm.str();
    }

    static const Rect TimeCodeROI;
    static const int DefaultDeltaNorm = 128;

    void findTransitionsSeconds( float start, float end, int deltaNorm = DefaultDeltaNorm )
    { findTransitions( floor( start * fps() ), ceil( end * fps() ), deltaNorm ); }

    void findTransitions( int start, int end, int deltaNorm = DefaultDeltaNorm )
    {
      start = std::max( 0, std::min( frameCount(), start ) );
      end = std::max( 0, std::min( frameCount(), end ) );

      capture.set( CV_CAP_PROP_POS_FRAMES, start );

      Mat prev;
      for( int at = start; at < end; ++at ) {
        Mat fullImage;
        capture >> fullImage;

        Mat timeCodeROI( fullImage, TimeCodeROI );

        if( prev.empty() ) {
          cv::cvtColor( timeCodeROI, prev, CV_BGR2GRAY );
          continue;
        }

        Mat curr;
        cv::cvtColor( timeCodeROI, curr, CV_BGR2GRAY );

        double norm = cv::norm( prev, curr, NORM_L2 );

#ifdef MAKE_NORMFILE
        normFile << at << " " << norm << endl; 
#endif

        if( norm > deltaNorm  ) {
          //cout << "Captured transition at frame " << at << endl;
          _transitions.insert( std::pair< int, TimecodeTransition >( at, TimecodeTransition( prev, curr ) ) );
        }

        prev = curr;
      }

      cout << "Have " << _transitions.size() << " transitions" << endl;

    }


    int normsSeconds( float start, float end, float **n )
    {  return norms( floor( start * fps() ), ceil( end * fps() ), n ); }

    int norms( int start, int end, float **n, int length = 0 )
    {
      start = std::max( 0, std::min( frameCount(), start ) );
      end = std::max( 0, std::min( frameCount(), end ) );

      int window = end-start;
      int len = std::max( length, window );

      *n = (float *)valloc( len * sizeof(float) );

      capture.set( CV_CAP_PROP_POS_FRAMES, start );

      Mat prev;
      for( int at = start; at < end; ++at ) {
        Mat fullImage;
        capture >> fullImage;

        Mat timeCodeROI( fullImage, TimeCodeROI );

        if( prev.empty() ) {
          cv::cvtColor( timeCodeROI, prev, CV_BGR2GRAY );
          continue;
        }

        Mat curr;
        cv::cvtColor( timeCodeROI, curr, CV_BGR2GRAY );

        (*n)[2*at] = cv::norm( prev, curr, NORM_L2 ); 
        (*n)[2*at+1] = 0.0f;

        prev = curr;
      }

      // Zero pad the remainder
      memset( &(*n[window]), 0, (len - window) * sizeof(float) );

      return window;
    }

    void dumpTransitions( const string &filename )
    {
      const int vspacing = 3, hspacing = 2;
      Mat canvas( Mat::zeros( Size( hspacing + 2 * TimeCodeROI.width, _transitions.size() * (TimeCodeROI.height + vspacing) ),
            CV_8UC1 ) );

      int i = 0;
      for( TransitionMap::iterator itr = _transitions.begin(); itr != _transitions.end(); ++itr, ++i ) {

        Mat beforeROI( canvas, Rect( 0, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );
        Mat  afterROI( canvas, Rect( TimeCodeROI.width + hspacing, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );

        itr->second.before.copyTo( beforeROI );
        itr->second.after.copyTo( afterROI );
      }

      imwrite( filename, canvas );
    }

  private:

    TransitionMap _transitions;
};

const Rect Video::TimeCodeROI = timeCodeROI_1920x1080;

int main( int argc, char **argv )
{
  string error;
  AlignmentOptions opts;
  if( opts.parseArgv( argc, argv, error ) != true ) {
    if( !error.empty() ) cout << error  << endl;
    exit(-1);
  }

#ifdef MAKE_NORMFILE
  normFile.open("norms.txt");
#endif

  Video video[2] = { Video( opts.video1 ), Video( opts.video2 ) };

  for( int i = 0; i < 2; ++i ) {
    if( !video[i].capture.isOpened() ) {
      cerr << "Can't open video " << i << endl;
      exit(-1);
    }

    cout << video[i].dump() << endl;

    video[i].findTransitionsSeconds( 0, opts.maxDelta + opts.window );

    stringstream filename;
    filename << "/tmp/transitions_" << i << ".png";

    video[i].dumpTransitions( filename.str() );

    if( i == 0 ) normFile << endl << endl;
  }

#ifdef MAKE_NORMFILE
  normFile.close();
#endif



  exit(0);
}



#ifdef FOURIER_APPROACH

  // Should be window floor'ed to nearest power of two or somesuch
  int windowLength = 128;
  float *norms0, *norms1;
  int count0 = video[0].norms( 0, windowLength, &norms0, windowLength*2 );
  int count1 = video[1].norms( 0, windowLength, &norms1, windowLength*2 );

  cout << "Got " << count0 << ", " << count1 << " norms" << endl;

  ffts_plan_t *forward0 = ffts_init_1d_real( 2*windowLength, -1 );
  ffts_plan_t *forward1 = ffts_init_1d_real( 2*windowLength, -1 );
  ffts_plan_t *backward = ffts_init_1d_real( 2*windowLength, 1 );

  // Real-to-complex transforms return (N/2 + 1) complex numbers
  // But I've zero-padded the input to 2*windowLength
  float fourierLength = (windowLength + 1);
  float __attribute__ ((aligned(32))) *fourier0 = (float *)valloc( fourierLength * 2 * sizeof(float) );
  float __attribute__ ((aligned(32))) *fourier1 = (float *)valloc( fourierLength * 2 * sizeof(float) );
  float __attribute__ ((aligned(32))) *fourierRes = (float *)valloc( fourierLength * 2 * sizeof(float) );
  float __attribute__ ((aligned(32))) *result = (float *)valloc( windowLength*2 * sizeof(float) );

  ffts_execute( forward0, norms0, fourier0 );
  ffts_execute( forward0, norms1, fourier1 );

  // Convolve
  float scale = 1.0 / fourierLength;
  for( int i = 0; i < fourierLength; ++i ) {
    fourierRes[ 2*i ]   = (fourier0[ 2*i ]*fourier1[2*i]   + fourier0[2*i+1]*fourier1[2*i+1]) * scale;
    fourierRes[ 2*i+1 ] = (fourier0[ 2*i+1 ]*fourier1[2*i] - fourier0[2*i]*fourier1[2*i+1]) * scale;
  }

  ffts_execute( backward, fourierRes, result );

  ofstream fst("correlation.txt");

  float max = 0;
  int maxIdx = -1;
  for( int i = 0; i < 2*windowLength; ++i ) {
    if( maxIdx < 0 || result[i] >  max ) {
      maxIdx = i;
      max = result[i];
    }

    fst << i << " " << result[i] << endl;
  }

  fst.close();

  cout << "Max occurs are index " << maxIdx << " value " << max << endl;

  ffts_free( forward0 );
  ffts_free( forward1 );
  ffts_free( backward );

  free( fourier0 );
  free( fourier1 );
  free( fourierRes );
  free( result );


  free( norms0 );
  free( norms1 );

  // Try a cross-correlation based approach (how will this deal with missing data?)


#endif

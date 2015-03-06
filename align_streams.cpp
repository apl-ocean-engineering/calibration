
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>

#ifdef USE_FFTS
#include <ffts.h>
#endif


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
  TimecodeTransition( int fr, const Mat &b, const Mat &a )
    : frame(fr), before( b.clone() ), after( a.clone() )
  {;}

  int frame;
  Mat before, after;

};

class Video
{
  public:

    typedef vector< TimecodeTransition > TransitionVec;

    Video( const string &file )
      : capture( file.c_str() ),filename( file )
    {
      // Should be more flexible about this..
      assert( (height() == 1080) && (width() == 1920) );
    }

    string filename;
    VideoCapture capture;

    const TransitionVec &transitions( void ) const;

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
      int length = end-start;

      capture.set( CV_CAP_PROP_POS_FRAMES, start );

      // First, sample the norms
      vector < float > norms(length, 0);
      float meanNorm = 0;
      vector < Mat > timecodes(length);

      Mat prev;
      for( int at = 0; at < length; ++at ) {
        Mat fullImage;
        capture >> fullImage;

        Mat timeCodeROI( fullImage, TimeCodeROI ), curr;
        cv::cvtColor( timeCodeROI, curr, CV_BGR2GRAY );

        curr.copyTo(timecodes[at]);

        if( prev.empty() ) {
          prev = curr;
          continue;
        }

        meanNorm += (norms[ at ] = cv::norm( prev, curr, NORM_L2 ));


        prev = curr;
      }

      // Gather statistics on the norms
      meanNorm /= length;

      float var = 0, stddev= 0;
      for( int i = 0; i < norms.size(); ++i ) var += pow( norms[i] - meanNorm, 2 );
      stddev = sqrt( var / (length-1) );

      cout << "Norm stats:: mean " << meanNorm << " stddev " << stddev << endl;

      int prevIdx = -1;
      int dtMean = fps();
      float dtStddev = 2.0;

      float pThreshold = 0.99;
      for( int i = 1; i < length; ++i ) {

        float p_norm = gsl_cdf_gaussian_P( norms[i] - meanNorm, stddev );
        float p_dt = 1.0;

        if( prevIdx >= 0 ) p_dt = gsl_cdf_gaussian_P( (i - prevIdx) - dtMean, dtStddev );

        float p = p_norm * p_dt;

        if( p_norm > pThreshold ) {
          //cout << "Captured transition at frame " << at+start << endl;
          _transitions.push_back( TimecodeTransition( i, timecodes[i-1], timecodes[i] ) );
          prevIdx = i;
        }


#ifdef MAKE_NORMFILE
        normFile << i << " " << norms[i] << ' ' << p_norm << ' ' << p_dt << ' ' << p << endl; 
#endif

      }

      cout << "Have " << _transitions.size() << " transitions" << endl;
    }

    typedef pair< int, int > IndexPair;


    IndexPair getSpan( int start, int length )
    {
      cout << "Getting span from " << start << " to " << start+length << endl;
      if( (start+length) > frameCount() ) { start = frameCount()-length;

        cout << "Adjusted span from " << start << " to " << length << endl;
      }

      int startIdx = 0;
      for( int i = 0; i < _transitions.size(); ++i )
        if( _transitions[i].frame > start ) { startIdx = i; break; }

      int endIdx = startIdx+1;
      for( int i = startIdx; i < _transitions.size(); ++i ) 
        if( _transitions[i].frame < (start+length) ) endIdx = i+1;


//      cout << "Excluded these transitios:" << endl;
//      for( int i = 0; i < startIdx; ++i ) cout << _transitions[i].frame << endl;
//      cout << endl;
//
//      cout << "Included there transitions"<< endl;
//      for( int i = startIdx; i < endIdx; ++i ) cout << _transitions[i].frame << endl;
//      cout <<endl;
//
//      cout << "Excluded there transitions"<< endl;
//      for( int i = endIdx; i < _transitions.size(); ++i ) cout << _transitions[i].frame << endl;
//      cout <<endl;


      return make_pair( startIdx, endIdx );
    }

    int alignWith( Video &other, float window, float maxDelta ) 
    {
      int maxDeltaFrames = floor(maxDelta * fps()),
          windowFrames = floor(window * fps());

      IndexPair otherSpan( other.getSpan( maxDeltaFrames, windowFrames ) );

      cout << "Other told to get from " << maxDeltaFrames << " " << windowFrames << endl;
      cout << " Got " << otherSpan.first << ' ' << otherSpan.second << endl;

      // Start with set of transitions from maxDelta to maxDelta+window (this is the maximum backwards shift on video1


      return 0;
    }

    void dumpTransitions( const string &filename )
    {
      const int vspacing = 3, hspacing = 2;
      Mat canvas( Mat::zeros( Size( 3 * (hspacing + TimeCodeROI.width), _transitions.size() * (TimeCodeROI.height + vspacing) ),
            CV_8UC1 ) );

      int i = 0;
      for( TransitionVec::iterator itr = _transitions.begin(); itr != _transitions.end(); ++itr, ++i ) {

        Mat beforeROI( canvas, Rect( 0, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );
        Mat  afterROI( canvas, Rect( TimeCodeROI.width + hspacing, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );
        Mat  diffROI( canvas, Rect( 2*(TimeCodeROI.width + hspacing), i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );

        itr->before.copyTo( beforeROI );
        itr->after.copyTo( afterROI );

        absdiff( itr->before, itr->after, diffROI );
      }

      imwrite( filename, canvas );
    }

  private:

    TransitionVec _transitions;
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

    video[i].findTransitionsSeconds( 0, 2*opts.maxDelta );

    stringstream filename;
    filename << "/tmp/transitions_" << i << ".png";

    video[i].dumpTransitions( filename.str() );

    if( i == 0 ) normFile << endl << endl;
  }

#ifdef MAKE_NORMFILE
  normFile.close();
#endif

  int offset = video[0].alignWith( video[1], opts.window, opts.maxDelta );



  exit(0);
}



#ifdef FOURIER_APPROACH

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

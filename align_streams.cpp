
#include <stdlib.h>

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

#include "video.h"
#include "synchronizer.h"

using namespace cv;
using namespace std;

#define MAKE_NORMFILE
#ifdef MAKE_NORMFILE
ofstream normFile;
#endif


struct AlignmentOptions
{
  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  AlignmentOptions( void )
    : window( 4.2 ), maxDelta( 5.0 ), lookahead(1.2), offset(0),  waitKey(0), offsetGiven(false)
  {;}


  float window, maxDelta, lookahead;
  int offset, waitKey;
  bool offsetGiven;

  string video1, video2;

  bool parseArgv( int argc, char **argv, string &msg )
  {
    static struct option long_options[] = {
      { "window", true, NULL, 'w' },
      { "wait-key", true, NULL, 'k' },
      { "max-delay", true, NULL, 'd'},
      { "offset", true, NULL, 'o'},
      { "lookahead", true, NULL, 'l'},
      { "help", false, NULL, '?' },
      { 0, 0, 0, }
    };

    int indexPtr;
    int optVal;
    while( (optVal = getopt_long( argc, argv, ":w:k:d:o:l:?", long_options, &indexPtr )) != -1 ) {
      switch(optVal) {
        case 'd':
          maxDelta = atof(optarg);
          break;
        case 'w':
          window = atof( optarg );
          break;
        case 'k':
          waitKey = atoi( optarg );
          break;
        case 'o':
          offset = atoi( optarg );
          offsetGiven = true;
          break;
        case 'l':
          lookahead = atof( optarg );
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

  VideoLookahead video[2] = { VideoLookahead( opts.video1, opts.lookahead ), VideoLookahead( opts.video2, opts.lookahead ) };
  TransitionVec transitions[2];

  for( int i = 0; i < 2; ++i ) {
    if( !video[i].capture.isOpened() ) {
      cerr << "Can't open video " << i << endl;
      exit(-1);
    }

    cout << video[i].dump() << endl;

    video[i].initializeTransitionStatistics( 0, 2*opts.maxDelta * video[i].fps(), transitions[i] );

    cout << "Found " << transitions[i].size() << " transitions" << endl;

    stringstream filename;
    filename << "/tmp/transitions_" << i << ".png";

    Video::dumpTransitions( transitions[i], filename.str() );

    if( i == 0 ) normFile << endl << endl;
  }

#ifdef MAKE_NORMFILE
  normFile.close();
#endif

  KFSynchronizer sync( video[0], video[1] );
  if( opts.offsetGiven )
    sync.setOffset( opts.offset );
  else
    sync.estimateOffset( transitions[0], transitions[1], 
        opts.window * video[0].fps(), opts.maxDelta * video[0].fps() );

  sync.rewind();
  sync.seek( 0, 151 );

  Mat img;
  while( sync.nextCompositeFrame( img )) {
    int ch;
    Mat shrunk;
    resize( img, shrunk, Size(), 0.5, 0.5 );
    imshow( "Composite", shrunk );
    ch = waitKey( opts.waitKey );

    if( ch == 'q' )
      break;
    else if (ch == ',')
      sync.scrub(-2);
    else if (ch == '[')
      sync.advanceToNextTransition( 0 );
    else if (ch == ']')
      sync.advanceToNextTransition( 1 );
    else if (ch == 'R')
      sync.rewind();
    else if (ch == 'l')
      sync.advanceOnly( 0 );
    else if (ch == 'r')
      sync.advanceOnly( 1 );

  }


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

  seek( start );

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

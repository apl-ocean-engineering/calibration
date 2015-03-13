#ifndef __VIDEO_H__
#define __VIDEO_H__

#include <stdlib.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>

#include <vector>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <getopt.h>

#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>

#include "file_utils.h"
#include "trendnet_time_code.h"

//#define MAKE_NORMFILE

using std::pair;
using std::string;

struct TimecodeTransition
{
  TimecodeTransition( int fr, const cv::Mat &b, const cv::Mat &a )
    : frame(fr), before( b.clone() ), after( a.clone() )
  {;}

  int frame;
  cv::Mat before, after;

};

typedef std::vector< TimecodeTransition > TransitionVec;
typedef std::map< int, TimecodeTransition > TransitionMap;


struct Gaussian {
  Gaussian()
    : mean(0.0), stddev(1.0) 
  {;}

  float mean, stddev;

  float p( int x ) {
    return gsl_cdf_gaussian_P( x - mean, stddev );
  }

  float pdf( int x ) {
    return gsl_ran_gaussian_pdf( x-mean, stddev );
  }
};

typedef pair< int, int > IndexPair;

class Video
{
  public:
    static const cv::Rect TimeCodeROI;

    Video( const string &file );

    string filename;
    cv::VideoCapture capture;

    float fps( void ) { return capture.get( CV_CAP_PROP_FPS ); }
    int frameCount( void ) { return capture.get( CV_CAP_PROP_FRAME_COUNT ); }
    int height( void ) { return capture.get( CV_CAP_PROP_FRAME_HEIGHT ); }
    int width( void ) { return capture.get( CV_CAP_PROP_FRAME_WIDTH ); }

    string dump( void ) 
    {
      std::stringstream strm;

      strm << "File " << filename << ": ";
      strm << width() << " x " << height() << ", ";
      strm << fps() << " fps.";

      return strm.str();
    }


//    void findTransitionsSeconds( float start, float end )
//    { findTransitions( floor( start * fps() ), ceil( end * fps() ) ); }

    virtual int frame( void ) { return capture.get( CV_CAP_PROP_POS_FRAMES ); }

    virtual void seek( int frame );
    void scrub( int offset ) { seek( frame()+offset ); }

    void rewind( void ) { seek( 0 ); }
    virtual bool read( cv::Mat &mat );

    void initializeTransitionStatistics( int start, int length, TransitionVec &transitions );

    bool detectTransition( float norm, int dt = -1);
    bool detectTransition( const cv::Mat &before, const cv::Mat &after, int dt = -1 );

    static void dumpTransitions( const TransitionVec &transitions, const string &filename );

    const TransitionMap &transitions( void ) const { return _transitions; }

    std::vector<int> transitionsAfter( int after );

  protected:

    Gaussian _distTimecodeNorm, _distDt;
    bool _transitionStatisticsInitialized;

    TransitionMap _transitions;

};


// !! Takes a copy of the image
struct CachedFrame
{
  public:
    CachedFrame( cv::Mat &img )
      : image( img.clone() ), _timecode()
    {;}

    cv::Mat image;

    void copyTo( cv::Mat &mat ) { image.copyTo(mat); }

    const cv::Mat &timecode( void );

  private:
      cv::Mat _timecode;
};

class VideoLookahead : public Video
{
  public:
    VideoLookahead( const string &file, float lookaheadSecs );

    virtual int frame( void ) { return  Video::frame() - _future.size(); }
    virtual void seek( int frame );
    virtual bool read( cv::Mat &mat );
    bool drop( void );

    int lookaheadFrames( void ) const { return _lookaheadFrames; }


  private:
    int _lookaheadFrames;
    std::queue< CachedFrame > _future;

int closestTransition( int frame );
};



#endif

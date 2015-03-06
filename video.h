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
using std::vector;
using std::string;

struct TimecodeTransition
{
  TimecodeTransition( int fr, const cv::Mat &b, const cv::Mat &a )
    : frame(fr), before( b.clone() ), after( a.clone() )
  {;}

  int frame;
  cv::Mat before, after;

};

typedef pair< int, int > IndexPair;

class Video
{
  public:

    typedef vector< TimecodeTransition > TransitionVec;

    Video( const string &file );

    string filename;
    cv::VideoCapture capture;

    const TransitionVec &transitions( void ) const { return _transitions; }
    const TransitionVec &transitions( void ) { return _transitions; }

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

    static const cv::Rect TimeCodeROI;
    static const int DefaultDeltaNorm = 128;

    void findTransitionsSeconds( float start, float end, int deltaNorm = DefaultDeltaNorm )
    { findTransitions( floor( start * fps() ), ceil( end * fps() ), deltaNorm ); }

    virtual int frame( void ) { return capture.get( CV_CAP_PROP_POS_FRAMES ); }

    virtual void seek( int frame );
    void scrub( int offset ) { seek( frame()+offset ); }

    void rewind( void ) { seek( 0 ); }
    virtual bool read( cv::Mat &mat );

    void findTransitions( int start, int end, int deltaNorm = DefaultDeltaNorm );

    IndexPair getSpan( int start, int length );

    bool shiftSpan( IndexPair &pair, int length, int direction );

    void dumpTransitions( const string &filename );
 
  private:

    TransitionVec _transitions;
};


class VideoLookahead : public Video
{
  public:
    VideoLookahead( const string &file, float lookaheadSecs );

    virtual int frame( void ) { return  Video::frame() - _queue.size(); }
    virtual void seek( int frame );
    virtual bool read( cv::Mat &mat );

  private:
    int _lookaheadFrames;
    std::queue< cv::Mat > _queue;
};



#endif

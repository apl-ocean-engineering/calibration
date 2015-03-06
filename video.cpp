
#include "video.h"

using namespace std;
using namespace cv;


const Rect Video::TimeCodeROI = timeCodeROI_1920x1080;

Video::Video( const string &file )
: capture( file.c_str() ),filename( file ),
  _distTimecodeNorm(), _distDt(), _transitionStatisticsInitialized( false )
{
  // Should be more flexible about this..
  assert( (height() == 1080) && (width() == 1920) );
}

bool Video::read( Mat &mat )
{
  return capture.read( mat ); 
}

void Video::seek( int frame )
{
  //  Incredibly inefficient but appears to be more reliable
  capture.open( filename );
  for( int i = 0; i < frame; ++i ) capture.grab();

  //capture.set( CV_CAP_PROP_POS_FRAMES, frame);
}

void Video::initializeTransitionStatistics( int start, int end, TransitionVec &transitions )
{
  start = std::max( 0, std::min( frameCount(), start ) );
  end = std::max( 0, std::min( frameCount(), end ) );
  int length = end-start;
  cout << "Looking over " << length << " frames";

  rewind();

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

  _distTimecodeNorm.mean = meanNorm;
  _distTimecodeNorm.stddev = stddev;
  cout << "Norm stats:: mean " << meanNorm << " stddev " << stddev << endl;

  _distDt.mean = fps();
  _distDt.stddev = 2.0;

  _transitionStatisticsInitialized = true;

  int prevIdx = -1;

  // Now identify the transitions
  transitions.clear();

  float pThreshold = 0.99;
  for( int i = 1; i < length; ++i ) {

    float p_norm = _distTimecodeNorm.p( norms[i] );
    float p_dt = 1.0;

    if( prevIdx >= 0 ) p_dt = _distDt.p( i - prevIdx );

    float p = p_norm * p_dt;

    if( p_norm > pThreshold ) {
      //cout << "Captured transition at frame " << at+start << endl;
      transitions.push_back( TimecodeTransition( i, timecodes[i-1], timecodes[i] ) );
      prevIdx = i;
    }


    cout << i << " " << norms[i] << ' ' << p_norm << ' ' << p_dt << ' ' << p << endl; 
#ifdef MAKE_NORMFILE
    normFile << i << " " << norms[i] << ' ' << p_norm << ' ' << p_dt << ' ' << p << endl; 
#endif

  }

  cout << "Have " << transitions.size() << " transitions" << endl;
}




void Video::dumpTransitions( const TransitionVec &transitions, const string &filename )
{
  const int vspacing = 3, hspacing = 2;
  Mat canvas( Mat::zeros( Size( 3 * (hspacing + TimeCodeROI.width), transitions.size() * (TimeCodeROI.height + vspacing) ),
        CV_8UC1 ) );

  int i = 0;
  for( TransitionVec::const_iterator itr = transitions.begin(); itr != transitions.end(); ++itr, ++i ) {

    Mat beforeROI( canvas, Rect( 0, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );
    Mat  afterROI( canvas, Rect( TimeCodeROI.width + hspacing, i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );
    Mat  diffROI( canvas, Rect( 2*(TimeCodeROI.width + hspacing), i * (TimeCodeROI.height + vspacing), TimeCodeROI.width, TimeCodeROI.height ) );

    itr->before.copyTo( beforeROI );
    itr->after.copyTo( afterROI );

    absdiff( itr->before, itr->after, diffROI );

    stringstream strm;
    strm << itr->frame;
    putText( diffROI, strm.str(), Point( 0, TimeCodeROI.height ), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,255,255) );
  }

  imwrite( filename, canvas );
}



//====


// Note setting _lookaheadFrames relies on capture() being initialized..
  VideoLookahead::VideoLookahead( const string &filename, float lookaheadSecs )
: Video( filename ), _lookaheadFrames( lookaheadSecs * fps() )
{;}

void VideoLookahead::seek( int dest )
{
  if( dest >= frame() && dest <= (frame() + _future.size()) ) {
    int drop = dest - frame();
    for( int i =0; i < drop && !_future.empty(); ++i ) _future.pop();

  } else {
    while( !_future.empty() ) _future.pop();

    Video::seek( dest );
  }
}

bool VideoLookahead::read( cv::Mat &mat ) 
{
  Mat framein;
  while( _future.size() < _lookaheadFrames && capture.read( framein ) )  {
    Mat foo;
    framein.copyTo( foo );
    _future.push( foo );
  }

  if( _future.size() > 0 ) {
    _future.front().copyTo( mat );
    _future.pop();
    return true;
  }

  return false;
}

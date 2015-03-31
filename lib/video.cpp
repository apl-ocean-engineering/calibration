
#include <iomanip>

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

void Video::initializeTransitionStatistics( int start, int length, TransitionVec &transitions )
{
  start = std::max( 0, std::min( frameCount(), start ) );
  cout << "Looking over frames " << start << " to " << start+length << endl;

  seek( start );

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

  _distDt.mean = 30; //fps();
  _distDt.stddev = 5.0;

  _transitionStatisticsInitialized = true;

  int prevIdx = -1;

  // Now identify the transitions
  transitions.clear();

  for( int i = 1; i < length; ++i ) {

    int dt = (prevIdx >= 0 ) ? (i-prevIdx) : -1;
    if( detectTransition( norms[i], dt ) ) {
      // The frames are numbered from 1 hence the "+1"
      transitions.push_back( TimecodeTransition( start+i+1, timecodes[i-1], timecodes[i] ) );
      prevIdx = i;
    }
  }


#ifdef MAKE_NORMFILE
  normFile << i << " " << norms[i] << ' ' << p_norm << ' ' << p_dt << ' ' << p << endl; 
#endif

cout << "Have " << transitions.size() << " transitions" << endl;
}


bool Video::detectTransition( float norm, int dt )
{
  float pThreshold = (dt > 0) ? 0.5 : 0.9;

  float p_norm = _distTimecodeNorm.p( norm );
  float p_dt = 1.0;

  if( dt > 0 ) {
    float dtErr = dt - _distDt.mean * std::max(1.0f,roundf( dt / _distDt.mean ));
    float dtScaled = dtErr / _distDt.stddev;
    p_dt = gsl_cdf_chisq_Q( dtScaled*dtScaled, 1 );
  }

  bool result =  (p_norm * p_dt) > pThreshold;
 // cout << dt << ' ' << std::setw(12) << norm << ' ' << std::setw(12) << p_norm << ' ' << std::setw(12) << p_dt << ' ' << std::setw(12) << (p_norm * p_dt) <<  (result ? " Y" : "") << endl; 
  return result;
}

bool Video::detectTransition( const Mat &before, const Mat &after, int dt )
{
  return detectTransition( cv::norm( before, after, NORM_L2 ), dt );
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


vector<int> Video::transitionsAfter( int after )
{
  vector<int> output;

  // Got to be a more efficient way to do this?
  for( TransitionMap::iterator itr = _transitions.begin(); itr != _transitions.end(); ++itr ) {
    if( itr->first > after ) output.push_back( itr->first );
  }

  return output;
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

int VideoLookahead::closestTransition( int frame )
{
  // Basically, the first key that's less than frame, starting from the back
  int ret = -1;

  for( TransitionMap::const_reverse_iterator itr = _transitions.rbegin(); itr != _transitions.rend(); ++itr ) {
    if( itr->second.frame < frame ) return itr->second.frame;
  }

  return ret;
}


bool VideoLookahead::read( cv::Mat &mat ) 
{
  Mat framein;
  while( _future.size() < _lookaheadFrames && capture.read( framein ) )  {

    // Look for transitions
    int frame = capture.get( CV_CAP_PROP_POS_FRAMES );
    int closest = closestTransition( frame );
    int dt = -1;
    if( closest >= 0 ) {
      int dt = (frame-closest);
      if( dt > 2*fps() ) dt = -1;
    }

    if( _future.size() > 0 ) {
      Mat prevTimecode( _future.back().timecode() );
      _future.push( framein );

      if( detectTransition( prevTimecode, _future.back().timecode(), dt ) ) {
        cout  << filename << ":  Believe there's a transition at frame " << frame << endl;
        _transitions.insert( make_pair( frame, TimecodeTransition( frame, prevTimecode, _future.back().timecode() ) ) );
      }
    } else {
      _future.push( framein );
    }
  }


  // Return the front of the queue
  if( _future.size() > 0 ) {
    _future.front().copyTo( mat );
    _future.pop();
    return true;
  }

  return false;
}

bool VideoLookahead::drop( void )
{
  Mat foo;
  return read( foo );
}

//== CachedFrame ===
const Mat &CachedFrame::timecode( void ) 
{
  if( _timecode.empty() ) {
    Mat timeCodeROI( image, Video::TimeCodeROI ), curr;
    cv::cvtColor( timeCodeROI, _timecode, CV_BGR2GRAY );
  }

  return _timecode;
}


#include "video.h"

using namespace std;
using namespace cv;


const Rect Video::TimeCodeROI = timeCodeROI_1920x1080;

Video::Video( const string &file )
: capture( file.c_str() ),filename( file )
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

void Video::findTransitions( int start, int end, int deltaNorm )
{
  start = std::max( 0, std::min( frameCount(), start ) );
  end = std::max( 0, std::min( frameCount(), end ) );
  int length = end-start;

  rewind();

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



IndexPair Video::getSpan( int start, int length )
{
  cout << "Getting span from " << start << " to " << start+length << endl;
  if( (start+length) > frameCount() ) { start = frameCount()-length;

    cout << "Adjusted span from " << start << " to " << length << endl;
  }

  int startIdx = 0;
  for( int i = 0; i < _transitions.size(); ++i )
    if( _transitions[i].frame > start ) { startIdx = i; break; }

  int endIdx = _transitions.size();
  for( int i = startIdx; i < _transitions.size(); ++i ) 
    if( _transitions[i].frame > (start+length) ) {endIdx = i; break;}


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

bool Video::shiftSpan( IndexPair &pair, int length, int direction )
{
  if( direction < 0 ) {
    if( pair.first == 0 ) return false;
    direction = -1;
  } else if (direction > 0 ) {
    if( pair.second == _transitions.size() ) return false;
    direction = +1;

  }
  pair.first += direction;
  pair.second = _transitions.size();
  int max = _transitions[pair.first].frame + length;
  for( int i = pair.first; i < _transitions.size(); ++i ) 
    if( _transitions[i].frame > max ) {pair.second = i; break;}

  return true;
}

void Video::dumpTransitions( const string &filename )
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
  if( dest >= frame() && dest <= (frame() + _queue.size()) ) {
    int drop = dest - frame();
    for( int i =0; i < drop && !_queue.empty(); ++i ) _queue.pop();

  } else {
    while( !_queue.empty() ) _queue.pop();

    Video::seek( dest );
  }
}

bool VideoLookahead::read( cv::Mat &mat ) 
{
  Mat framein;
  while( _queue.size() < _lookaheadFrames && capture.read( framein ) )  {
    Mat foo;
    framein.copyTo( foo );
    _queue.push( foo );
  }

  if( _queue.size() > 0 ) {
    _queue.front().copyTo( mat );
    _queue.pop();
    return true;
  }

  return false;
}


#include "synchronizer.h"

using namespace std;
using namespace cv;

const float Synchronizer::Scale = 1.0;

  Synchronizer::Synchronizer( Video &v0, Video &v1 )
: _video0( v0 ), _video1( v1 ), _offset( 0 )
{;}


void Synchronizer::rewind( void )
{
  if( _offset < 0 ) {
    _video0.seek( -_offset );
    _video1.seek( 0 );
  } else {
    _video0.seek( 0 );
    _video1.seek( _offset );
  }
  cout << "Rewind to frames: " << _video0.frame() << ' ' << _video1.frame() << endl;
}

bool Synchronizer::seek( int which, int dest )
{
  if( which == 0 ) {
    int dest1 = dest + _offset;

    if( dest >= 0 && dest < _video0.frameCount() && 
        dest1 >= 0 && dest1 < _video1.frameCount() ) {
      _video0.seek(dest);
      _video1.seek(dest1);
      return true;
    } 

  } else {
    return seek( 0, dest - _offset );
  }

  return  false;
}

bool Synchronizer::scrub( int offset )
{  int dest0 = _video0.frame() + offset;
  return seek( 0, dest0 );
}


bool Synchronizer::advanceToNextTransition( int which )
{
  int current = (which == 0) ? _video0.frame() : _video1.frame();
  Video &vid( (which == 0) ? _video0 : _video1 );
  const TransitionMap &transitions( vid.transitions() );

  if( transitions.size() == 0 )
    return false;
  else 
  {
    if( transitions.begin()->first > current ) {
      seek( which, transitions.begin()->first );
      cout << "Advancing o " << which << " to frame " << transitions.begin()->first << endl;
      return true;
    } 

    if( transitions.size() > 1 ) {
      TransitionMap::const_iterator itr = transitions.begin(), prev = itr;
    itr++;
    for( ; itr != transitions.end(); ++itr, ++prev ) {
      if( (prev->first <= current) && (itr->first > current) ) {
          seek( which , itr->first );
          cout << "Advancing o " << which << " to frame " << itr->first << endl;
          return true;
        }
      }
    }
  }

  return false;
}

Size Synchronizer::compositeSize( void )
{ 
  return  Size( Scale*(_video0.width() + _video1.width()), Scale*std::max(_video0.height(), _video1.height()) ); 
}

void Synchronizer::advanceOnly( int which )
{
  if( which == 0 ) {
    _offset--;
    _video1.scrub(-1);
  } else {
    _offset++;
    _video0.scrub(-1);
  }
}

bool Synchronizer::nextCompositeFrame( Mat &img )
{

  img.create( compositeSize(), CV_8UC3 );

  Mat video0ROI( img, Rect( 0, 0, Scale*_video0.width(), Scale*_video0.height() ) );
  Mat video1ROI( img, Rect( Scale*_video0.width(), 0, Scale*_video1.width(), Scale*_video1.height() ) );


  Mat frame;
  if( _video0.read( frame ) ) 
    if( Scale != 1.0 )
      resize( frame, video0ROI, video0ROI.size() );
    else
      frame.copyTo( video0ROI );
  else return false;

  if( _video1.read(frame )  )
    if( Scale != 1.0 )
      resize( frame, video1ROI, video1ROI.size() );
    else
      frame.copyTo( video1ROI );
  else return false;

  cout << "Frames: " << _video0.frame() << ' ' << _video1.frame() <<  ' ' << _offset << endl;

  return true;
}

//---------------------------------------------------------------------------
// Tools for estimating initial offset
//---------------------------------------------------------------------------

IndexPair Synchronizer::getSpan( const TransitionVec &transitions, int start, int length )
{
  cout << "Getting span from " << start << " to " << start+length << endl;
  //if( (start+length) > frameCount() ) { start = frameCount()-length;

  //  cout << "Adjusted span from " << start << " to " << length << endl;
  //}

  int startIdx = 0;
  for( int i = 0; i < transitions.size(); ++i )
    if( transitions[i].frame > start ) { startIdx = i; break; }

  int endIdx = transitions.size();
  for( int i = startIdx; i < transitions.size(); ++i ) 
    if( transitions[i].frame > (start+length) ) {endIdx = i; break;}


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

bool Synchronizer::shiftSpan( const TransitionVec &transitions, IndexPair &pair, int length, int direction )
{
  if( direction < 0 ) {
    if( pair.first == 0 ) return false;
    direction = -1;
  } else if (direction > 0 ) {
    if( pair.second == transitions.size() ) return false;
    direction = +1;

  }
  pair.first += direction;
  pair.second = transitions.size();
  int max = transitions[pair.first].frame + length;
  for( int i = pair.first; i < transitions.size(); ++i ) 
    if( transitions[i].frame > max ) {pair.second = i; break;}

  return true;
}

float Synchronizer::compareSpans( const TransitionVec &thisTransitions,  IndexPair &thisSpan, 
    const TransitionVec &otherTransitions, IndexPair &otherSpan )
{
  cout << "======================" << endl;
  cout << "this span:  " << thisSpan.first << ' ' << thisSpan.second << endl;
  for( int i = thisSpan.first; i < thisSpan.second; ++i ) {
    if( i > thisSpan.first ) 
      cout << thisTransitions[i].frame << ' ' << thisTransitions[i].frame - thisTransitions[i-1].frame << endl;
    else
      cout << thisTransitions[i].frame << ' ' <<endl;
  }

  cout << "other span: " << otherSpan.first << ' ' << otherSpan.second << endl;
  for( int i = otherSpan.first; i < otherSpan.second; ++i ) {
    if( i > otherSpan.first ) 
      cout << otherTransitions[i].frame << ' ' << otherTransitions[i].frame - otherTransitions[i-1].frame << endl;
    else
      cout << otherTransitions[i].frame << ' ' <<endl;
  }


  if( (thisSpan.second - thisSpan.first) != (otherSpan.second - otherSpan.first) ) {
    cout << "Spans are different lengths, using shorter of the two " << endl;
  }
  int  length = std::min( thisSpan.second-thisSpan.first, otherSpan.second-otherSpan.first );

  float total = 0.0;
  for( int i = 0; i < length; ++i ) {
    total += norm( thisTransitions[ thisSpan.first+i ].before, otherTransitions[ otherSpan.first+i ].before, NORM_L2 );
    total += norm( thisTransitions[ thisSpan.first+i ].after, otherTransitions[ otherSpan.first+i ].after, NORM_L2 );
  }
  total /= length;

  return total;
}

struct OffsetResult {
  OffsetResult( IndexPair vid0, IndexPair vid1 )
    : v0( vid0 ), v1( vid1 )
  {;}

  IndexPair v0, v1;
};

int Synchronizer::estimateOffset( const TransitionVec &trans0,  const TransitionVec &trans1, float windowFrames, float maxDeltaFrames ) 
{
  // TODO:  Currently assumes both videos have same FPS
  map <float, OffsetResult> results;

  IndexPair thisSpan( getSpan( trans0,  0, windowFrames ) );
  IndexPair otherSpan( getSpan(  trans1, maxDeltaFrames, windowFrames ) );

  cout << "Getting span with max delta " << maxDeltaFrames << "  length " << windowFrames << endl;
  cout << " Got " << thisSpan.first << ' ' << thisSpan.second << endl;
  cout << " Got " << otherSpan.first << ' ' << otherSpan.second << endl;


  // Start with set of transitions from maxDelta to maxDelta+window (this is the maximum backwards shift on video1)

  do {
    float result = compareSpans( trans0, thisSpan, trans1,  otherSpan );
    cout << "    result: " << result << endl;
    results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
  } while( shiftSpan( trans1, otherSpan, windowFrames, -1 ) ) ;

  // Now start shifting my span forward
  while( shiftSpan(  trans0, thisSpan, windowFrames, +1 ) ) {
    float result = compareSpans( trans0, thisSpan, trans1,  otherSpan );
    cout << "    result: " << result << endl;
    results.insert( make_pair( result, OffsetResult( thisSpan, otherSpan ) ) );
  }

  OffsetResult best = results.begin()->second;

  // Calculate offset from end of spans...
  // Need to handle case where spans are different length
  int dt = std::min( best.v0.second-best.v0.first, best.v1.second-best.v1.first ) - 1;
  _offset = trans1[ dt ].frame - trans0[ dt ].frame;

  cout << "Best alignment has score " << results.begin()->first << endl;
  cout << "With frames " << trans0[ best.v0.second-1 ].frame << " " << trans1[ best.v1.second-1 ].frame << endl;
  cout << "With video1 offset to video0 by " << _offset << endl;

  return _offset;
}






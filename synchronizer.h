#ifndef __SYNCHRONIZER_H__
#define __SYNCHRONIZER_H__

#include <opencv2/core/core.hpp>

#include "video.h"


class Synchronizer
{
  public:

        static const float Scale;

    Synchronizer( Video &v0, Video &v1 );

    void setOffset( int offset ) { _offset = offset; }

    void rewind( void );
    bool seek( int which, int dest );
    bool scrub( int offset );

    bool advanceToNextTransition( const TransitionVec &transition, int which );
    void advanceOnly( int which );

    cv::Size compositeSize( void );
    bool nextCompositeFrame( cv::Mat &img );
 
    // Tools for estimating initial offset
    IndexPair getSpan( const TransitionVec &transition,  int start, int length );
    bool shiftSpan( const  TransitionVec &transition, IndexPair &pair, int length, int direction );
    float compareSpans( const TransitionVec &thisTransitions,  IndexPair &thisSpan,  
        const TransitionVec &otherTransitions, IndexPair &otherSpan );

    struct OffsetResult {
      OffsetResult( IndexPair vid0, IndexPair vid1 )
        : v0( vid0 ), v1( vid1 )
      {;}

      IndexPair v0, v1;
    };

    int estimateOffset(  const TransitionVec &transitions0, const TransitionVec &transitions1,
        float window, float maxDelta ) ;
 

  private:
    Video & _video0, &_video1;

    int _offset;


};


#endif


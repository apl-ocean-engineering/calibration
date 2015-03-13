#ifndef __SYNCHRONIZER_H__
#define __SYNCHRONIZER_H__

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

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

    bool advanceToNextTransition( int which );
    void advanceOnly( int which );

    cv::Size compositeSize( void );
    virtual bool nextSynchronizedFrames( cv::Mat &video0, cv::Mat &video1 );
    virtual bool nextCompositeFrame( cv::Mat &img );

    void compose( const cv::Mat &img0, cv::Mat &img1, cv::Mat &composite, float scale = Scale );

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

    virtual int estimateOffset(  const TransitionVec &transitions0, const TransitionVec &transitions1,
        float window, float maxDelta, int seekTo = 0 ) ;

    int bootstrap( float windowFrames, float maxDeltaFrames, int seekTo = 0 );

  protected:
    Video & _video0, &_video1;

    int _offset;


};


class SynchroKalmanFilter
{
  public:
    SynchroKalmanFilter( int depth );

    int predict( void );
    int update( int obs, int future );

    void setOffset( int offset );

  private:

    int depth( void ) const { return _state.rows(); }

    Eigen::VectorXd _state;
    Eigen::MatrixXd _cov;

    Eigen::MatrixXd _f, _q;
    Eigen::Matrix<double,1,1> _r;

};


class KFSynchronizer : public Synchronizer 
{
  public:
    KFSynchronizer( VideoLookahead &v0, VideoLookahead &v1 );

    virtual bool nextSynchronizedFrames( cv::Mat &video0, cv::Mat &video1 );

    virtual int estimateOffset(  const TransitionVec &transitions0, const TransitionVec &transitions1,
        float window, float maxDelta, int seekTo ) ;

  private:
    VideoLookahead &_lvideo0, &_lvideo1;
    int _lastObs[2];

    SynchroKalmanFilter _kf;
};





#endif


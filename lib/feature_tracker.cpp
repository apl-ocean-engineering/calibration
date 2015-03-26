
#include <algorithm>
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>

#include "feature_tracker.h"

namespace AplCam {

  using namespace cv;
  using namespace std;

  const float FeatureTracker::_dropRadius = 5;
  const float FeatureTracker::_patchRadius = 5;
  const int FeatureTracker::_maxMisses = 5;
  const int FeatureTracker::_maxTracks = 1500;

  typedef Matx<float,2,4> Matx24f;
  typedef Matx<float,4,2> Matx42f;
  typedef Matx<float,2,1> Matx21f;

  FeatureTracker::FeatureTracker( void )
    : _previous(), _tracks()
  {;}


  struct TxDistanceLessThan {
    TxDistanceLessThan( const Point2f &pt, float r )
      : _pt(pt), _r2( r*r ) {;}
    const Point2f &_pt;
    float _r2;

    bool operator()( const KeyPoint &other )
    {
      Point2f d = other.pt - _pt;
      return ( (d.x*d.x + d.y*d.y ) < _r2 );
    }
  };

  void FeatureTracker::update( Mat &img, vector< KeyPoint > &kps, Mat &drawTo, float scale )
  {
    bool doDraw = (drawTo.empty() == false);
    float s = ( (scale == 0.0) ? 1.0 : 1.0/scale);
    Rect imageRect( 0, 0, img.size().width, img.size().height );

    vector< list<KeyPointTrack>::iterator > dropList;

    int kpsInitially = kps.size(), kpsTooClose = 0;

    // Attempt to update each currently known track
    for( list<KeyPointTrack>::iterator itr = _tracks.begin();
        itr != _tracks.end(); ++itr ) {
      KeyPointTrack &track( *itr );

      Point2f prev = track.pt();
      if( doDraw ) {
        for( deque< Point2f >::reverse_iterator ritr = track.history.rbegin();
            ritr != track.history.rend(); ++ritr ) {
          if( ritr == track.history.rbegin() ) prev = (*ritr);
          circle( drawTo, s*(*ritr), 5, Scalar( 0, 0, 255), 1 );
          line( drawTo, s*prev, s*(*ritr), Scalar( 0, 0, 255), 1 );
          prev = (*ritr);
        }
      }


      Location pred = track.predict( );
      if( doDraw ) circle( drawTo, s*pred.pt, 5, Scalar( 0,255,0), 2 );

      // Nice expensive square root..
      float searchXw = std::max( 5, (int)ceil( 2 * sqrt( pred.cov.x )) ),
            searchYw = std::max( 5, (int)ceil(2 * sqrt( pred.cov.y )) );
      Rect searchArea( pred.pt.x - searchXw, pred.pt.y - searchYw, 2 * searchXw, 2 * searchYw );
      searchArea &= imageRect;

      Mat roi( img, searchArea );

      Point2f match;
      bool matched = track.search( roi, match ); 

      match  = match + Point2f(searchArea.x, searchArea.y);
      //cout << pred.pt.x << " " << pred.pt.y << " -- " << match.x << " " << match.y << endl;

      if( matched && !tooNearEdge( img, match ) ) {
        if( doDraw ) {
          circle( drawTo, s * match, 5, Scalar( 255,0,0), 2 );
          line( drawTo, s*prev, s*match, Scalar(0,0,255), 1 );
        }

        roi = patchROI( img, match );
        track.update( roi, match );
        track.missed = 0;

        // If there's been a successful match, drop any keypoints which are close by
        int before = kps.size();
        vector< KeyPoint >::iterator newEnd = std::remove_if( kps.begin(), kps.end(), TxDistanceLessThan( match, _dropRadius ) );
        if( newEnd != kps.end() ) {
        kps.erase( newEnd, kps.end() );
        kpsTooClose += before - kps.size();
        track.refeatured = std::min(10, track.refeatured+1 );;
        } else {
          --track.refeatured;
        }

        if( track.refeatured < 0 ) dropList.push_back( itr );


      } else {
        ++track.missed;
      }

      if( track.missed > _maxMisses ) dropList.push_back( itr );
    }

    // Delete any dropped tracks
    for( vector< list<KeyPointTrack>::iterator >::iterator itr = dropList.begin();
        itr != dropList.end(); ++itr ) 
      _tracks.erase( *itr );

    // Process any new keypoints
    // for now, new points are just added
    int count = 0, kpsTooNearEdge = 0;
    for( vector< KeyPoint >::iterator itr = kps.begin(); itr != kps.end() && _tracks.size() < _maxTracks; ++itr ) {
      const KeyPoint &kp( *itr );

      if( tooNearEdge(  img, kp.pt ) ) { ++kpsTooNearEdge;  continue; }

      ++count;
      Mat roi = patchROI( img, kp.pt );
      _tracks.push_back( KeyPointTrack( roi, new DecayingVelocityMotionModel( kp.pt ) ) );
    }

    cout << "From " << kps.size() << "/" << kpsInitially << " remain.  Added " << count << " dropped " << kpsTooClose << " as too close, " << kpsTooNearEdge << " too near edge, and dropped " << dropList.size() << " tracks for a total of " << _tracks.size() << endl;

    // Maintain an archival copy
    img.copyTo( _previous );
  }


  void FeatureTracker::drawTracks( Mat &img, float scale )
  {
    float s = 1.0/scale;

    for( list<KeyPointTrack>::iterator itr = _tracks.begin();
        itr != _tracks.end(); ++itr ) {
      KeyPointTrack &track( *itr );

      circle( img, s*track.pt(), 5, Scalar( 0, 255, 0), 1 );

    }
  }


  //===========================================================================

  FeatureTracker::KeyPointTrack::KeyPointTrack( const Mat &patch, MotionModel *model )
    : _patch(), _motionModel(model), refeatured(3)
  {
    patch.convertTo( _patch, CV_32FC1, 1.0/255.0 );
  }

  FeatureTracker::KeyPointTrack::~KeyPointTrack( void )
  {
    //if( _motionModel != NULL ) _motionModel;
  }

  bool FeatureTracker::KeyPointTrack::search( const Mat &roi, Point2f &match )
  {
    bool success = false;
    Mat roif;
    roi.convertTo( roif, CV_32FC1, 1.0/255.0 );

    if( roif.size().width < _patch.size().width ||
        roif.size().height < _patch.size().height ) return false;

    Mat result;
    matchTemplate( roif, _patch, result, CV_TM_CCORR_NORMED );

    Point minLoc, maxLoc;
    double mina, maxa;
    minMaxLoc( result, &mina, &maxa, &minLoc, &maxLoc );

    // Check result with heuristics here

    // Adjust match so it is relative to the upper left corner of roi
    match = maxLoc + Point( (_patch.size().width-1)/2, (_patch.size().height-1)/2);
    success = true;

    return success;
  }


  void FeatureTracker::KeyPointTrack::update( const Mat &patch, const Point2f &position )
  {
    patch.convertTo( _patch, CV_32FC1, 1.0/255.0 );

    history.push_front( position );
    while( history.size() > 10 ) history.pop_back();

    _motionModel->update( position );
  }


  //===========================================================================
  //
  // Basic 2dof Kalman Filter
  //
  // Model is:
  //
  // x_t+1 = x_t + v_t dt
  // v_t+1 = alpha v_t 
  //
  //


  FeatureTracker::DecayingVelocityMotionModel::DecayingVelocityMotionModel( const Point2f &x_0 )
    : MotionModel(), alpha(0.7),_state( x_0.x, x_0.y, 0, 0 ), _cov( Matx44f::eye() )
  {
    _cov(0,0) = _cov(1,1) = 5;
    _cov(2,2) = _cov(3,3) = 25;
  }

  FeatureTracker::Location FeatureTracker::DecayingVelocityMotionModel::predict( void  )
  {
    // Assume fixed rate operation
    //   Do the matrix-based approach for now.  Could expand later if necessary
    //
    // State change matrix
    const Matx44f F( 1, 0, 1, 0,
                     0, 1, 0, 1,
                     0, 0, alpha, 0,
                     0, 0, 0, alpha );
    // Process noise
    const Matx44f Q( 0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 25, 0,
                     0, 0, 0, 25 );

    _state = F * _state;
    _cov = F * _cov * F.t() + Q;

return Location( _state(0), _state(1), _cov(0,0), _cov(1,1) );
  }

  void FeatureTracker::DecayingVelocityMotionModel::update( const Point2f &l )
  {
    // Measurement matrix
    const Matx24f H( 1, 0, 0, 0,
                     0, 1, 0, 0 );

    // Observation covariances
    Matx21f y = Matx21f( l.x, l.y ) - H*_state;
    Matx22f R = 2 * Matx22f::eye();

    Matx22f S = H * _cov * H.t() + R;
    Matx42f K = _cov * H.t() * S.inv();

    _state += K * y;
    _cov = (Matx44f::eye() - K*H)*_cov;

  }

}

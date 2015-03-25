
#include <opencv2/imgproc/imgproc.hpp>

#include "feature_tracker.h"

namespace AplCam {

  using namespace cv;

  const float FeatureTracker::_dropRadius = 5;
  const float FeatureTracker::_patchRadius = 5;
  const int FeatureTracker::_maxMisses = 5;

  typedef Matx<float,2,4> Matx24f;
  typedef Matx<float,4,2> Matx42f;
  typedef Matx<float,2,1> Matx21f;

  FeatureTracker::FeatureTracker( void )
    : _previous(), _tracks()
  {;}


  struct TxDropCloseTo {
    TxDropCloseTo( const Point2f &pt, float r2 )
      : _pt(pt), _r2( r2 ) {;}
    const Point2f &_pt;
    float _r2;

    bool operator()( const KeyPoint &other )
    {
      Point2f d( other.pt - _pt );
      return ( (d.x*d.x + d.y*d.y ) > _r2 );
    }
  };

  void FeatureTracker::update( Mat &img, vector< KeyPoint > &kps, Mat &drawTo, float scale )
  {
    bool doDraw = (drawTo.empty() == false);
    float s = ( (scale == 0.0) ? 1.0 : 1.0/scale);
    Rect imageRect( 0, 0, img.size().width, img.size().height );

    vector< vector<KeyPointTrack>::iterator > dropList;

    // Attempt to update each currently known track
    for( vector<KeyPointTrack>::iterator itr = _tracks.begin();
        itr != _tracks.end(); ++itr ) {
      KeyPointTrack &track( *itr );

      if( doDraw ) circle( drawTo, s*track.pt(), 5, Scalar( 0, 255, 0), 1 );

      Location pred = track.predict( );

      if( doDraw ) circle( drawTo, s * pred.pt, 5, Scalar( 0,0,255), 1 );

      // Nice expensive square root..
      float searchXw = 2 * sqrt( pred.cov.x ),
            searchYw = 2 * sqrt( pred.cov.y );
      Rect searchArea( pred.pt.x - searchXw, pred.pt.y - searchYw, 2 * searchXw, 2 * searchYw );
      searchArea &= imageRect;

      Mat roi( img, searchArea );

      Point2f match;
      bool matched = track.search( roi, match ); 

      match  = match + Point2f(searchArea.x, searchArea.y);


      if( matched && !tooNearEdge( img, match ) ) {
        if( doDraw ) circle( drawTo, s * match, 5, Scalar( 255,0,0), 1 );

        roi = patchROI( img, match );
        track.update( roi, match );
        track.missed = 0;

        // If there's been a successful match, drop any keypoints which are close by
        std::remove_if( kps.begin(), kps.end(), TxDropCloseTo( match, _dropRadius * _dropRadius ) );

      } else {
        ++track.missed;
      }

      if( track.missed > _maxMisses ) dropList.push_back( itr );
    }

    // Delete any dropped tracks
    for( vector< vector<KeyPointTrack>::iterator >::iterator itr = dropList.begin();
        itr != dropList.end(); ++itr ) 
      _tracks.erase( *itr );

    // Process any new keypoints
    // for now, new points are just added
    for( vector< KeyPoint >::iterator itr = kps.begin(); itr != kps.end(); ++itr ) {
      const KeyPoint &kp( *itr );

      if( tooNearEdge(  img, kp.pt ) ) continue;

      Mat roi = patchROI( img, kp.pt );
      _tracks.push_back( KeyPointTrack( roi, new DecayingVelocityMotionModel( kp.pt ) ) );
    }

    // Maintain an archival copy
    img.copyTo( _previous );
  }


  void FeatureTracker::drawTracks( Mat &img, float scale )
  {
    float s = 1.0/scale;

    for( vector<KeyPointTrack>::iterator itr = _tracks.begin();
        itr != _tracks.end(); ++itr ) {
      KeyPointTrack &track( *itr );

      circle( img, s*track.pt(), 5, Scalar( 0, 255, 0), 1 );

    }
  }


  //===========================================================================

  FeatureTracker::KeyPointTrack::KeyPointTrack( const Mat &patch, MotionModel *model )
    : _patch(), _motionModel(model)
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

    Mat result;
    matchTemplate( roif, _patch, result, CV_TM_CCORR_NORMED );

    Point minLoc, maxLoc;
    double mina, maxa;
    minMaxLoc( result, &mina, &maxa, &minLoc, &maxLoc );

    // Check result with heuristics here

    match = maxLoc;
    success = true;

    return success;
  }


  void FeatureTracker::KeyPointTrack::update( const Mat &patch, const Point2f &position )
  {
    patch.convertTo( _patch, CV_32FC1, 1.0/255.0 );

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

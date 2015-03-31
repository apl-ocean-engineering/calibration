
#ifndef __FEATURE_TRACKER_H__
#define __FEATURE_TRACKER_H__

#include <vector>
#include <deque>
#include <list>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace AplCam {

  using cv::Mat;
  using cv::KeyPoint;
  using cv::Point2f;

  using std::vector;

  class FeatureTracker {
    public:

      FeatureTracker( void );

      void update( Mat &img, vector< KeyPoint > &kps, Mat &drawTo, float scale = 1.0 );

      void drawTracks( Mat &img, float scale = 1.0 );

    protected:


      struct Location {
        Location( const Point2f &_pt, const Point2f &_cov ) 
          : pt( _pt ), cov( _cov ) {;}
        Location( float x, float y, float covx, float covy )
          : pt( x,y ), cov( covx, covy ) {;}

        cv::Point2f pt;
        cv::Point2f cov;
      };

      struct MotionModel {
        MotionModel( void ) {;};
        virtual ~MotionModel() {;}

        virtual Location predict( void ) = 0;
        virtual void update( const Point2f &loc ) = 0;

        virtual Point2f pt( void ) const = 0;
      };

      struct DecayingVelocityMotionModel : public MotionModel {
        DecayingVelocityMotionModel( const Point2f &x_0 );
        virtual ~DecayingVelocityMotionModel() {;}

        virtual Location predict( void );
        virtual void update( const Point2f &loc );

        virtual Point2f pt( void ) const { return Point2f( _state(0), _state(1) ); }

        float alpha;
        cv::Vec4f _state;
        cv::Matx44f _cov;
      };


      struct KeyPointTrack {
        KeyPointTrack( const Mat &patch, MotionModel *model );
        ~KeyPointTrack( void );

        Location predict( void  ) { return _motionModel->predict(); }
        bool search( const Mat &roi, Point2f &match );
        void update( const Mat &patch, const Point2f &position );

        Point2f pt( void ) const { return Point2f( _motionModel->pt() ); }

        std::shared_ptr<MotionModel> _motionModel;
        Mat _patch;
        std::deque< Point2f > history;

        int missed, refeatured;
      };

      Mat patchROI( Mat &img, const Point2f &center )
      {
        return Mat( img, cv::Rect( center.x - _patchRadius, center.y - _patchRadius,
              2*_patchRadius + 1, 2*_patchRadius+1 ) );
      }

      bool tooNearEdge( const Mat &img, const Point2f &pt )
      {
        return ( pt.x - _patchRadius < 0 || pt.y - _patchRadius < 0 ||
            pt.x + _patchRadius >= img.size().width ||
            pt.y + _patchRadius >= img.size().height );
      }

      Mat  _previous;
      std::list< KeyPointTrack > _tracks;

      static const float _dropRadius;
      static const float _patchRadius;
      static const int _maxMisses;
      static const int _maxTracks;
  };

}


#endif

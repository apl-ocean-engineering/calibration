#ifndef __APRILTAG_DETECTOR_H__
#define __APRILTAG_DETECTOR_H__

#include <opencv2/core/core.hpp>

#ifdef USE_TBB
#include "tbb/tbb.h"
#endif

#include "AplCam/board.h"
#include "AplCam/detection/detection.h"
#include "AplCam/detection_db.h"

#include "detector.h"

namespace camera_calibration {

using cv::Mat;
using cv::Point2f;

using AplCam::DetectionDb;


struct AprilTagDetectorFunctor {
 public:
  AprilTagDetectorFunctor( FrameVec_t &frames, DetectionDb &db, Board *board )
      : _frames( frames ), _db(db), _board( *board), timingData()
  {;}

  FrameVec_t &_frames;
  DetectionDb &_db;
  Board &_board;
  TimingDataVec_t timingData;

#ifdef USE_TBB
  // Splitting constructor for TBB
  AprilTagDetectorFunctor( AprilTagDetectorFunctor &other, tbb::split )
      : _frames( other._frames ), _db( other._db ), _board( other._board ), timingData()
  {;}


  // Interestingly, this function isn't really const, as it
  // modifies class members...
  void operator()( const tbb::blocked_range<size_t> &r ) const
  {
    size_t end = r.end();
    for( size_t i = r.begin(); i != end; ++i ) {
#else
      void operator()( ) const
      {

        size_t end = _frames.size();
        for( size_t i = 0; i < end; ++i ) {
#endif
          Detection *detection = NULL;
          Frame &p( _frames[i]);

          //cout << "Extracting from " << p.frame << ". ";

          //              Mat grey;
          //              cvtColor( p.img, grey, CV_BGR2GRAY );

          int64 before = cv::getTickCount();
          detection = _board.detectPattern( p.img );
          int64 elapsed = cv::getTickCount() - before;

          //cout << p.frame << ": " << detection->size() << " features" << endl;

          // Trust the thread-safety of kyotocabinet
          _db.save( p.frame, *detection);

          int sz = detection->size();
          delete detection;

          //timingData.push_back( make_pair( sz, elapsed ) );
        }
      }



#ifdef USE_TBB
      void join( const AprilTagDetectorFunctor &other )
      {
        std::copy( other.timingData.begin(), other.timingData.end(), back_inserter( timingData ) );
      }
#endif

    };
  }


#endif

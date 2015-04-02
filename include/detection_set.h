
#ifndef __DETECTION_SET_H__
#define __DETECTION_SET_H__

#include <map>
#include <vector>
#include <algorithm>
#include <string>

#include "detection.h"

namespace AplCam {

  using std::map;
  using std::vector;
  using std::string;

  class DetectionSet {
    public:

      typedef vector< Detection * > DetectionMap;

      DetectionSet() 
        : _name(""), _detections(), _frames()
      {;}

      ~DetectionSet( void )
      {
        for( DetectionMap::iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) delete (*itr);
      }

      size_t size( void ) const { return _detections.size(); }

      void addDetection( DetectionDb &db, const int frame )
      { 
        Detection *detection = db.load( frame );
        if( detection ) {
          _detections.push_back( detection );
          _frames.push_back( frame );
        }
      }


      //==
      int imageObjectPoints( ImagePointsVecVec &imgPts, ObjectPointsVecVec &objPts ) const
      {
        int count = 0;
        for( DetectionMap::const_iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) {
          const Detection &det( **itr );
          imgPts.push_back( det.points );
          objPts.push_back( det.corners );
          count += det.corners.size();
        }

        return count;
      }

      struct SetElement {
        SetElement( int f, Detection *det )
          : frame( f ), _det(det) {;}

        int frame;
        Detection *_det;

        operator Detection &( void ) { return *_det; }
        operator Detection *( void ) { return _det; }
      };


      SetElement operator[]( unsigned int i)
      {
        return SetElement( _frames[i], _detections[i] );
      }


      const string &setName( const string &n )
      { _name = n; return _name; }

      const string &name( void ) const
      { return _name; }

      const vector<int> &frames( void ) const 
      { return _frames; }


    protected:
      string _name;

      DetectionMap _detections;
      vector< int > _frames;
  };

}





#endif

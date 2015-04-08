
#ifndef __DETECTION_SET_H__
#define __DETECTION_SET_H__

#include <map>
#include <vector>
#include <algorithm>
#include <string>

#include "detection.h"
#include "detection_db.h"

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
        addDetection( db.load( frame ), frame );
      }

      void addDetection( Detection *detection, const int frame )
      { 
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


      // Inefficient, for now
      ObjectPointsVecVec objectPoints( void ) const {
        ObjectPointsVecVec v;
        for( DetectionMap::const_iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) 
          v.push_back( (*itr)->corners );
        return v;
      }

      ImagePointsVecVec imagePoints( void ) const {
        ImagePointsVecVec v;
        for( DetectionMap::const_iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) 
          v.push_back( (*itr)->points );
        return v;
      }

      RotVec rvecs( void ) const {
        RotVec v;
        for( DetectionMap::const_iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) 
          v.push_back( (*itr)->rot );
        return v;
      }

      TransVec tvecs( void ) const {
        TransVec v;
        for( DetectionMap::const_iterator itr = _detections.begin(); itr != _detections.end(); ++itr ) 
          v.push_back( (*itr)->trans );
        return v;
      }


      struct ConstSetElement {
        ConstSetElement( int f, Detection *det )
          : frame( f ), _det(det) {;}

        int frame;
        const Detection *_det;

        operator const Detection &( void ) { return *_det; }
        operator const Detection *( void ) { return _det; }
      };


      ConstSetElement operator[]( unsigned int i)  const
      {
        return ConstSetElement( _frames[i], _detections[i] );
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

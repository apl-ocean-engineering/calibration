
#ifndef __DISPARITY_DB_H__
#define __DISPARITY_DB_H__

#include <string>
using std::string;

#include <opencv2/core.hpp>
using cv::Mat;


#include <kchashdb.h>

class SequentialDb {
public:
SequentialDb();
  SequentialDb( const string &filename, bool writable = false );

  bool open( const string &filename, bool writable = false );
  void close( void );

  bool setFps( double fps );

  bool isOpen( void ) const
    { return _isOpen; }

  kyotocabinet::BasicDB::Error error( void )
    { return _db.error(); }

protected:

  kyotocabinet::TreeDB _db;
  bool _isOpen;

  static const string FpsKey;
};

class DisparityMatDb : public SequentialDb {
public:
DisparityMatDb();
  DisparityMatDb( const string &filename, bool writable = false );

  bool save( int frame, const Mat &disp );

};

class PointCloudDb : public SequentialDb {
public:
PointCloudDb();
  PointCloudDb( const string &filename, bool writable = false );

  bool save( int frame, const Mat &disp );
};


#endif

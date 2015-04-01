

#ifndef __VIDEO_SPLITTER_H__
#define __VIDEO_SPLITTER_H__

#include <kchashdb.h>

#include "detection_set.h"

#include "video_splitter_opts.h"


namespace AplCam {

  namespace VideoSplitters {

    using kyotocabinet::DB;

    class VideoSplitter {
      public:
        VideoSplitter()
        {;}


        virtual void generate( DetectionDb &db, DetectionSet &set ) = 0;
    };


    class AllVideoSplitter : public VideoSplitter {
      public:
        AllVideoSplitter( void ) {;}

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          DB::Cursor *cur = db.cursor();
          string key;
          while( cur->get_key( &key, false ) ) set.addDetection( db, stoi(key) );
          delete cur;

        }
    };

    class RandomVideoSplitter : public VideoSplitter {
      public:
        RandomVideoSplitter( const RandomSplitterOpts &opts ) 
          : _count( opts.count ) 
        {;}

        long int _count;

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          vector< string > keys;

          DB::Cursor *cur = db.cursor();
          cur->jump();

          string key;
          while( cur->get_key( &key, false ) ) keys.push_back(key);
          delete cur;

          long int c = std::max( _count, (long int)keys.size() );

          std::random_shuffle( keys.begin(), keys.end() );
          keys.resize( c );

          for( vector< string >::iterator itr = keys.begin(); itr != keys.end(); ++itr ) set.addDetection( db, stoi(key) );
        }
    };

    class IntervalVideoSplitter : public VideoSplitter {
      public:
        IntervalVideoSplitter( const IntervalSplitterOpts &opts )
          : _offset( opts.offset ), _interval( opts.interval )
        {;}

        int _offset, _interval;

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          for( int i = _offset; i < db.maxKey(); i += _interval ) 
            set.addDetection( db,  i );
        }

    };


  }
}


#endif

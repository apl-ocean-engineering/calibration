

#ifndef __VIDEO_SPLITTER_H__
#define __VIDEO_SPLITTER_H__

#include <kchashdb.h>

#include "bits_to_hex.h"
#include "random.h"
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
          while( cur->get_key( &key, true ) ) set.addDetection( db, stoi(key) );
          set.setName( "all" );

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
          while( cur->get_key( &key, true) ) keys.push_back(key);
          delete cur;

          long int c = std::min( _count, (long int)keys.size() );

          std::random_shuffle( keys.begin(), keys.end(), unaryRandom );
          keys.resize( c );

          for( vector< string >::iterator itr = keys.begin(); itr != keys.end(); ++itr ) {
            set.addDetection( db, stoi(*itr) );
          }

          stringstream strm;
          strm << "random(" << _count << ")_" << intsToHex( set.frames() );
          set.setName( strm.str() );
        }
    };

    class IntervalVideoSplitter : public VideoSplitter {
      public:
        IntervalVideoSplitter( const IntervalSplitterOpts &opts )
          : _start( opts.start ), _end(opts.end), _interval( opts.interval )
        {;}

        int _start, _end, _interval;

        virtual void generate( DetectionDb &db, DetectionSet &set )
        {
          int e = std::min( _end, db.maxKey() );

          for( int i = _start; i < e; i += _interval ) 
            set.addDetection( db,  i );

          stringstream strm;
          strm << "interval(" << _start << "," << _interval << ',' << _end << ")_" << intsToHex( set.frames() );
          set.setName( strm.str() );
        }

    };


  }
}


#endif

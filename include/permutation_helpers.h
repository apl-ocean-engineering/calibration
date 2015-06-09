

#ifndef __PERMUTATION_HELPERS_H__
#define __PERMUTATION_HELPERS_H__



struct RandomKeyCounter {
  RandomKeyCounter( int i ) : _count(i) {;}
  int _count;

  bool operator()( const string &key )
  {
    int c;
    int found = sscanf( key.c_str(), "random(%d)", &c );
    if( found != 1 ) return false;

    return c == _count;
  }
};


struct IntervalKeyFinder {
  // Ignore end
  IntervalKeyFinder( int start, int interval ) 
    : _start( start ), _interval( interval ) {;}

  int _start, _interval;

  bool operator()( const string &key )
  {
    int s, i;
    int found = sscanf( key.c_str(), "interval(%d,%d", &s, &i );
    if( found != 2 ) return false;

    return (s==_start && i == _interval);
  }
};





#endif



#include <cryptopp/sha.h>
#include <cryptopp/hex.h>
#include <cryptopp/files.h>

#include <stdlib.h>
#include <sys/stat.h>

#include <string>

#include <boost/filesystem.hpp>

using namespace std;

namespace fs = boost::filesystem;

bool file_exists( const string &infile ) 
{
  return fs::is_regular_file( infile );
  //struct stat buffer;   
  //return (stat(infile.c_str(), &buffer) == 0 );
}

bool directory_exists( const string &infile ) 
{
  return fs::is_directory( infile );
  //struct stat buffer;   
  //return (stat(infile.c_str(), &buffer) == 0  && (buffer.st_mode & S_IFDIR));
}

void mkdir_p( const string &dir )
{
  fs::path p( dir );
  fs::create_directories( p.parent_path() );

//  char tmp[256];
//  char *p = NULL;
//  size_t len;
//
//  snprintf(tmp, sizeof(tmp),"%s",dir.c_str() );
//  len = strlen(tmp);
//
//  bool finalSep = (tmp[len - 1] == '/');
//
//  for(p = tmp + 1; *p; p++)
//    if(*p == '/') {
//      *p = 0;
//      mkdir(tmp, S_IRWXU);
//      *p = '/';
//    }
//
//  if( finalSep ) {
//    tmp[len-1] = 0;
//    mkdir(tmp, S_IRWXU);
//    tmp[len-1] = '/';
//  }
}


const string fileHashSHA1( const string &filename )
{
  string out;
  CryptoPP::SHA1 hash;
  CryptoPP::FileSource( filename.c_str(), true, 
      new CryptoPP::HashFilter( hash,
        new CryptoPP::HexEncoder( new CryptoPP::StringSink( out ), true ) ) );

  return out;
}

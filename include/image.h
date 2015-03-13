#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <string>

#include <opencv2/core/core.hpp>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;


class Image {
  public:
    Image( const std::string &filename, cv::Mat &img )
      : _fileName( filename ), _img( img )
    {;}

    const std::string &fileName( void ) const { return _fileName.string(); }
    const cv::Mat    &img( void )      const { return _img; }

    const cv::Size size( void ) const { return _img.size(); }

    const std::string &hash( void ) const;

    std::string basename( void ) const
    {
      return _fileName.filename().string();
      //size_t sep = _fileName.find_last_of( '/' );
      //if( sep == std::string::npos )
      //  return _fileName;

      //return std::string( _fileName, sep+1 );
    }

  private:

    fs::path _fileName;
    cv::Mat _img;

    mutable std::string _hash;
};


#endif

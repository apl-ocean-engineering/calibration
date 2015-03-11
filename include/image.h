#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <string>

#include <opencv2/core/core.hpp>


class Image {
  public:
    Image( const std::string &filename, cv::Mat &img )
      : _fileName( filename ), _img( img )
    {;}

    const std::string &fileName( void ) const { return _fileName; }
    const cv::Mat    &img( void )      const { return _img; }

    const cv::Size size( void ) const { return _img.size(); }

    std::string basename( void )
    {
      size_t sep = _fileName.find_last_of( '/' );
      if( sep == std::string::npos )
        return _fileName;

      return std::string( _fileName, sep+1 );
    }

  private:

    std::string _fileName;
    cv::Mat _img;
};


#endif

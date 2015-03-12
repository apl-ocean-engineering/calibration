
#ifndef __BOARD_H__
#define __BOARD_H__

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#ifdef USE_APRILTAGS
#include <AprilTags/TagDetector.h>
#include <AprilTags/TagFamily.h>
#include <AprilTags/Tag36h11.h>
#endif

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID, APRILTAGS };

// Forward decl
struct Detection;

class Board {
  public:
    Board( Pattern pat, int w, int h, float squares, const std::string &nam )
      : name(nam), pattern(pat), width(w), height(h), squareSize( squares )
    {;}

    std::string name;
    Pattern pattern;
    int width, height;
    float squareSize;

    cv::Size size( void ) const { return cv::Size( width,height ); }

    virtual Detection *detectPattern( const cv::Mat &gray, std::vector< cv::Point2f > &pointbuf );
    static Board *load( const std::string &infile, const std::string &name );

    std::string patternString( void ) const {
      switch(pattern)
      {
        case CHESSBOARD:
          return "chessboard";
          break;
        case CIRCLES_GRID:
          return "circles_grid";
          break;
        case ASYMMETRIC_CIRCLES_GRID:
          return "asym_circles_grid";
          break;
        case APRILTAGS:
          return "apriltags";
          break;
      }
      return "";
    }


  protected:

    virtual void loadCallback( cv::FileStorage &fs ) {;}

  private:
};

#ifdef USE_APRILTAGS
class AprilTagsBoard : public Board {
  public:

    AprilTagsBoard( int w, int h, float squares, const std::string &name )
      : Board(  APRILTAGS, w, h, squares, name ), _tagCode( AprilTags::tagCodes36h11 )
    {;} 

    virtual Detection *detectPattern( const cv::Mat &gray, vector< cv::Point2f > &pointbuf );

    bool find( const int id, cv::Point2i &xy  ) const;
    cv::Point3f worldLocation( const cv::Point2i &xy ) const;

  protected:

    virtual void loadCallback( cv::FileStorage &fs );

  private:

    cv::Mat _ids;

    AprilTags::TagCodes _tagCode;
};
#endif


#endif

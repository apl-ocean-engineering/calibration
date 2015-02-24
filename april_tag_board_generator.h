
#include <opencv2/core/core.hpp>
#include <vector>

#include <AprilTags/TagFamily.h>

#ifndef __APRIL_TAG_BOARD_GENERATOR_H__
#define __APRIL_TAG_BOARD_GENERATOR_H__

using cv::Point3f;
using cv::Point2f;
using cv::Mat;
using cv::Size;
using std::vector;

using AprilTags::TagFamily;

namespace cv {

class AprilTagBoardGenerator 
{
public:
    double sensorWidth;
    double sensorHeight;
    size_t squareEdgePointsNum;
    double min_cos;
    mutable double cov;

    Size _patternSize;
    double _tagSize;
    double _tagSpacing;

    int rendererResolutionMultiplier;

    AprilTagBoardGenerator(const Size& patternSize = Size(8, 6), const double tagSize = 1, const double tagSpacing = 1 );

    Mat generate(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, vector<Point2f>& corners) const;

    vector<Point3f> worldPoints( void );

private:

    // Calculate the field of view given the camera matrix, image size
    // and the sensor size (which is fixed)
    Point2d fieldOfView( const Mat &camMat, const Size &imgSize );;;;

    void generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const;

    Mat generateBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
        const Point3f& zero, const Point3f& pb1, const Point3f& pb2,
        float sqWidth, float sqHeight, const vector<Point3f>& whole, vector<Point2f>& corners) const;
    void generateBasis(Point3f& pb1, Point3f& pb2) const;

    Point3f generateChessBoardCenter(const Mat& camMat, const Size& imgSize) const;

    TagFamily _tagFamily;
    Mat _tags;

    Mat rvec, tvec;
};

}

#endif

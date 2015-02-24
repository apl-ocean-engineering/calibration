
#include <opencv2/core/core.hpp>
#include <vector>

#ifndef __APRIL_TAG_BOARD_GENERATOR_H__
#define __APRIL_TAG_BOARD_GENERATOR_H__

using cv::Point3f;
using cv::Point2f;
using cv::Mat;
using cv::Size;
using std::vector;


namespace cv {

class AprilTagBoardGenerator 
{
public:
    double sensorWidth;
    double sensorHeight;
    size_t squareEdgePointsNum;
    double min_cos;
    mutable double cov;

    Size patternSize;
    double tagSize;
    double tagSpacing;

    int rendererResolutionMultiplier;

    ChessBoardGenerator(const Size& patternSize = Size(8, 6), const double tagSize, const double tagSpacing );

    Mat operator()(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, vector<Point2f>& corners) const;

private:

    void generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const;

    Mat generateBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
        const Point3f& zero, const Point3f& pb1, const Point3f& pb2,
        float sqWidth, float sqHeight, const vector<Point3f>& whole, vector<Point2f>& corners) const;
    void generateBasis(Point3f& pb1, Point3f& pb2) const;

    Point3f generateChessBoardCenter(const Mat& camMat, const Size& imgSize) const;

    Mat rvec, tvec;
};

}

#endif

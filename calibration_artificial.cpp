#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

#include "chess_board_generator.h"

using namespace cv;
using namespace std;

static void help()
{
    printf( "\nThis code generates an artificial camera and artificial chessboard images,\n"
            "and then calibrates. It is basically test code for calibration that shows\n"
            "how to package calibration points and then calibrate the camera.\n"
            "Usage:\n"
            "./calibration_artificial\n\n");
}
namespace cv
{



const Size imgSize(800, 600);
const Size brdSize(8, 7);
const size_t brds_num = 20;

template<class T> ostream& operator<<(ostream& out, const Mat_<T>& mat)
{
    for(int j = 0; j < mat.rows; ++j)
        for(int i = 0; i < mat.cols; ++i)
            out << mat(j, i) << " ";
    return out;
}

}


int main( int argc, char **argv )
{
    help();
    cout << "Initializing background...";
    Mat background(imgSize, CV_8UC3);
    randu(background, Scalar::all(32), Scalar::all(255));
    GaussianBlur(background, background, Size(5, 5), 2);
    cout << "Done" << endl;

    cout << "Initializing chess board generator...";
    ChessBoardGenerator cbg(brdSize);
    cbg.rendererResolutionMultiplier = 4;
    cout << "Done" << endl;

    /* camera params */
    Mat_<double> camMat(3, 3);
    camMat << 300., 0., background.cols/2., 0, 300., background.rows/2., 0., 0., 1.;

    Mat_<double> distCoeffs(1, 5);
    distCoeffs << 1.2, 0.2, 0., 0., 0.;

    cout << "Generating chessboards...";
    vector<Mat> boards(brds_num);
    vector<Point2f> tmp;
    for(size_t i = 0; i < brds_num; ++i)
        cout << (boards[i] = cbg(background, camMat, distCoeffs, tmp), i) << " ";
    cout << "Done" << endl;

    vector<Point3f> chessboard3D;
    for(int j = 0; j < cbg.cornersSize().height; ++j)
        for(int i = 0; i < cbg.cornersSize().width; ++i)
            chessboard3D.push_back(Point3i(i, j, 0));

    /* init points */
    vector< vector<Point3f> > objectPoints;
    vector< vector<Point2f> > imagePoints;

    cout << endl << "Finding chessboards' corners...";
    for(size_t i = 0; i < brds_num; ++i)
    {
        cout << i;
        namedWindow("Current chessboard"); imshow("Current chessboard", boards[i]); waitKey(100);
        bool found = findChessboardCorners(boards[i], cbg.cornersSize(), tmp);
        if (found)
        {
            imagePoints.push_back(tmp);
            objectPoints.push_back(chessboard3D);
            cout<< "-found ";
        }
        else
            cout<< "-not-found ";

        drawChessboardCorners(boards[i], cbg.cornersSize(), Mat(tmp), found);
        imshow("Current chessboard", boards[i]); waitKey(1000);
    }
    cout << "Done" << endl;
    cvDestroyAllWindows();

    Mat camMat_est;
    Mat distCoeffs_est;
    vector<Mat> rvecs, tvecs;

    cout << "Calibrating...";
    double rep_err = calibrateCamera(objectPoints, imagePoints, imgSize, camMat_est, distCoeffs_est, rvecs, tvecs);
    cout << "Done" << endl;

    cout << endl << "Average Reprojection error: " << rep_err/brds_num/cbg.cornersSize().area() << endl;
    cout << "==================================" << endl;
    cout << "Original camera matrix:\n" << camMat << endl;
    cout << "Original distCoeffs:\n" << distCoeffs << endl;
    cout << "==================================" << endl;
    cout << "Estimated camera matrix:\n" << (Mat_<double>&)camMat_est << endl;
    cout << "Estimated distCoeffs:\n" << (Mat_<double>&)distCoeffs_est << endl;

    return 0;
}





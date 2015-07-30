#ifndef __GRABCUT_H__
#define __GRABCUT_H__


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using cv::InputArray;
using cv::InputOutputArray;
using cv::Rect;

void graphCut( InputArray img, InputOutputArray mask, Rect rect,
               InputOutputArray bgdModel, InputOutputArray fgdModel,
                           int iterCount, int mode = cv::GC_EVAL );

#endif

#ifndef CV_UTILS_H
#define CV_UTILS_H

#include <optional>
#include <string>

#include <lemon/dim2.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <opencv2/opencv.hpp>

namespace mfplan {

void viz_path(std::optional<ompl::geometric::PathGeometric> path, cv::Mat img);

void viz_point(lemon::dim2::Point<double> coords, cv::Mat img);

// convert numeric image type to string for debug
std::string getImgType(int imgTypeInt);

} // namespace mfplan

#endif

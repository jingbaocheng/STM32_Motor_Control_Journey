#pragma once

#include <vector>
#include <opencv2/core.hpp>


struct CalibrationPoint
{
    double u;
    double v;
    double X;
    double Y;
};


struct CalibrationData
{
    double a;
    double b;
    double c;

    double d;
    double e;
    double f;

    bool calibrated;
};


class CoordinateMapper
{
public:
    CoordinateMapper();

    bool Calibrate(
        const std::vector<CalibrationPoint>& points
    );

    cv::Point2d PixelToMachine(
        double u,
        double v
       
    ) const;

private:
    CalibrationData data_;
};

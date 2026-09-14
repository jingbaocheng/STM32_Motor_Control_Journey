#include "CoordinateMapper.h"
#include <stdexcept>

CoordinateMapper::CoordinateMapper()
{
    data_.a = 0.0;
    data_.b = 0.0;
    data_.c = 0.0;

    data_.d = 0.0;
    data_.e = 0.0;
    data_.f = 0.0;

    data_.calibrated = false;
}
bool CoordinateMapper::Calibrate(
    const std::vector<CalibrationPoint>& points)
{
    if (points.size() < 3)
    {
        data_.calibrated = false;
        return false;
    }

    int n = static_cast<int>(points.size());

    cv::Mat A(n, 3, CV_64F);
    cv::Mat X(n, 1, CV_64F);
    cv::Mat Y(n, 1, CV_64F);

    for (int i = 0; i < n; ++i)
    {
        A.at<double>(i, 0) = points[i].u;
        A.at<double>(i, 1) = points[i].v;
        A.at<double>(i, 2) = 1.0;

        X.at<double>(i, 0) = points[i].X;
        Y.at<double>(i, 0) = points[i].Y;
    }
    cv::Mat pX;
    cv::Mat pY;

    bool okX = cv::solve(
        A,
        X,
        pX,
        cv::DECOMP_SVD
    );

    bool okY = cv::solve(
        A,
        Y,
        pY,
        cv::DECOMP_SVD
    );
    if (!okX || !okY)
    {
        data_.calibrated = false;
        return false;
    }

    data_.a = pX.at<double>(0, 0);
    data_.b = pX.at<double>(1, 0);
    data_.c = pX.at<double>(2, 0);

    data_.d = pY.at<double>(0, 0);
    data_.e = pY.at<double>(1, 0);
    data_.f = pY.at<double>(2, 0);
    data_.calibrated = true;
    return true;
}
cv::Point2d CoordinateMapper::PixelToMachine(
    double u,
    double v) const
{
    if (!data_.calibrated)
    {
        throw std::runtime_error(
            "CoordinateMapper is not calibrated."
        );
    }

    double X = data_.a * u
        + data_.b * v
        + data_.c;

    double Y = data_.d * u
        + data_.e * v
        + data_.f;

    return cv::Point2d(X, Y);
}
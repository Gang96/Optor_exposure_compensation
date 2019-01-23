#include "optor_stereo_visensor_ros/stereo_visensor_cam.h"

double gDevirativePixel(unsigned char pixelValue, vector<double> &gDevirative, double preExposureTime)
{
    double ans;
    for (size_t i = 0; i < gDevirative.size(); ++i)
    {
        ans += gDevirative[i] * pow((double)pixelValue, (int)i);
    }
    return 1 / (ans * preExposureTime);
}

int exposureCalculation(double preExposureTime, cv::Mat latestImg, double gamma, vector<double> &g)
{
    int row = latestImg.rows;
    int col = latestImg.cols;
    cv::Mat W = cv::Mat::zeros(row, col, CV_64FC1);
    int numOfPixels = row * col;
    int k = 5;
    int pS = 0.8 * numOfPixels;
    double sum = 0;
    int i = 0;
    double const pi = 3.14159265;
    int dx;
    int dx2;
    int dy;
    int dy2;
    cv::Mat result;

    // Calculate the weights
    for (cv::MatIterator_<uchar> it = W.begin<uchar>(); it != W.end<uchar>(); ++it)
    {
        if (i <= pS)
            *it = pow(sin(pi * i / (2 * pS)), k);
        else
            *it = pow(sin(pi / 2 - pi * (i - pS) / 2 / (numOfPixels - pS)), k);
        i++;
        sum += *it;
    }
    W = W / sum;

    // Devirative of g
    vector<double> gDevirative;
    for (int i = 1; i < g.size(); i++)
        gDevirative.push_back(i * g[i]);

    // Devirative of the Gradient Magnitude
    cv::Mat GradMagDevi = cv::Mat::zeros(row, col, CV_64FC1);
    for (int i = 0; i < row; i++)
        for (int j = 0; j < col; j++)
        {
            if (i != row - 1)
            {
                unsigned char pixCur = latestImg.at<uchar>(i, j);
                unsigned char pixNex = latestImg.at<uchar>(i + 1, j);
                dx = pixNex - pixCur;
                dx2 = gDevirativePixel(pixNex, gDevirative, preExposureTime) - gDevirativePixel(pixCur, gDevirative, preExposureTime);
            }
            else
            {
                unsigned char pixCur = latestImg.at<uchar>(i, j);
                unsigned char pixPre = latestImg.at<uchar>(i - 1, j);
                dx = pixCur - pixPre;
                dx2 = gDevirativePixel(pixCur, gDevirative, preExposureTime) - gDevirativePixel(pixPre, gDevirative, preExposureTime);
            }
            if (j != col - 1)
            {
                unsigned char pixCur = latestImg.at<uchar>(i, j);
                unsigned char pixNex = latestImg.at<uchar>(i, j + 1);
                dy = pixNex - pixCur;
                dy2 = gDevirativePixel(pixNex, gDevirative, preExposureTime) - gDevirativePixel(pixCur, gDevirative, preExposureTime);
            }
            else
            {
                unsigned char pixCur = latestImg.at<uchar>(i, j);
                unsigned char pixPre = latestImg.at<uchar>(i, j - 1);
                dy = pixCur - pixPre;
                dy2 = gDevirativePixel(pixCur, gDevirative, preExposureTime) - gDevirativePixel(pixPre, gDevirative, preExposureTime);
            }
            GradMagDevi.at<float>(i, j) = 2 * (dx * dx2 + dy * dy2);
        }
    cv::multiply(W, GradMagDevi, result);
    cv::Scalar ans = cv::sum(result);
    double exposureTime = gamma * ans.val[0] + preExposureTime;
    return exposureTime;
}

//初始化程序：文献参考——HDR文献第13篇参考文献
//通过对一系列的图片和每个图片对应的曝光时间进行处理得到曝光响应函数
#include "optor_stereo_visensor_ros/stereo_visensor_cam.h"


int weightingFunction(int pixelValue, int pixelValueMax = 255, int pixelValueMin = 0)
{
    if (pixelValue <= 0.5 * (pixelValueMax + pixelValueMin))
        return pixelValue - pixelValueMin;
    else
        return pixelValueMax - pixelValue;
}
vector<double> responseRecovery(vector<cv::Mat> &imgs, vector<double> &deltaT, double lambda = 0.5)
{

    int numOfImgs = imgs.size();
    int row = imgs[0].rows;
    int col = imgs[0].cols;
    int numOfPixels = row * col;
    int n = 256;
    cout << "Calculating the matrixs" << endl;
    cv::Mat A = cv::Mat::zeros(numOfImgs * numOfPixels + n + 1, n + numOfPixels, CV_8SC1);//内存溢出，修改重点
    cout << "A matrix done..." << endl;
    cv::Mat b = cv::Mat::zeros(numOfImgs * numOfPixels + n + 1, 1, CV_64FC1);
    cout << "b matrix done..." << endl;

    //include the data fitting equations
    int k = 0;
    int i = 0;
    int wij;
    for (int j = 0; j++; j < numOfImgs)
        for (cv::MatIterator_<uchar> it = imgs[j].begin<uchar>(); it != imgs[j].end<uchar>(); ++it)
        {
            i++;
            wij = weightingFunction(*it);
            A.at<double>(k, *it) = wij;
            A.at<double>(k, n + i) = -wij;
            b.at<double>(k, 0) = wij * -log(deltaT[j]);
            k++;
        }

    //fix the curve by setting its middle value to 0
    A.at<double>(k, 128) = 1;
    k++;

    //include the smoothness equations
    for (int i = 0; i++; i <= n - 1)
    {
        A.at<double>(k, i) = weightingFunction(i);
        A.at<double>(k, i + 1) = -2 * weightingFunction(i);
        A.at<double>(k, i + 2) = weightingFunction(i);
        k++;
    }
    
    //solve the system using SVD
    cv::Mat invA;
    cv::Mat invB;
    cv::Mat x;
    cv::invert(A, invA, cv::DECOMP_SVD);
    cv::multiply(invA, b, x);

    //fit g with a tenth order polynomial
    //运用多项式去拟合响应函数g，详见fit类的定义
    Fit fit;
    vector<double> X;
    vector<double> Y;
    for (int i = 0; i++; i < n)
    {
        X.push_back(i);
        Y.push_back(*x.ptr<double>(i));
    }
    fit.polyfit(X, Y, 10);//拟合阶数为10
    vector<double> g;
    fit.getFactor(g);
    return g;
}

#ifndef SUPERPOINTEXTRACTOR_H
#define SUPERPOINTEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <ExtractorNode.h>
#include "BaseExtractor.h"
#include <ort_utility/ort_utility.hpp>
#include <ort-superpoint/SuperPoint.hpp>

namespace ORB_SLAM3
{
typedef std::pair<std::vector<cv::KeyPoint>, cv::Mat> KeyPointAndDesc;

class SuperPointExtractor: public BaseExtractor
{
public:
    SuperPointExtractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST, Ort::SuperPoint* superPoint);

    int operator()( cv::InputArray _image, cv::InputArray _mask,
                    std::vector<cv::KeyPoint>& _keypoints,
                    cv::OutputArray _descriptors, std::vector<int> &vLappingArea);

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }

    std::vector<cv::Mat> mvImagePyramid;

protected:

    void ComputePyramid(cv::Mat image);
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints, std::vector<cv::Mat>& allDescriptors);    
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);

    std::vector<cv::Point> pattern;

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int iniThFAST;
    int minThFAST;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;

    Ort::SuperPoint* superPoint;
};

} //namespace ORB_SLAM

#endif


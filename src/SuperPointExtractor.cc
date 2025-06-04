#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <iostream>

#include "SuperPointExtractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM3
{
    SuperPointExtractor::SuperPointExtractor(int _nfeatures, float _scaleFactor, int _nlevels,
                               int _iniThFAST, int _minThFAST, Ort::SuperPoint* _superPoint):
            nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels),
            iniThFAST(_iniThFAST), minThFAST(_minThFAST), superPoint(_superPoint)
    {
        mvScaleFactor.push_back(1.0f);
        mvLevelSigma2.push_back(1.0f);

        mvInvScaleFactor.push_back(1.0f/mvScaleFactor[0]);
        mvInvLevelSigma2.push_back(1.0f/mvLevelSigma2[0]);

        // This is for orientation
        // pre-compute the end of a row in a circular patch
        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }

    int SuperPointExtractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                                  OutputArray _descriptors, std::vector<int> &vLappingArea)
    {
        if(_image.empty())
            return -1;

        Mat image = _image.getMat();
        assert(image.type() == CV_8UC1 );

        KeyPointAndDesc result = superPoint->inference(*superPoint, image, 4, 0.015, true, 2);
        const int nkeypoints = result.first.size();

        // Add border to coordinates and scale information
        // const int minBorderX = EDGE_THRESHOLD-3;
        // const int minBorderY = minBorderX;
        // const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[0];
        // for(int i=0; i<nkeypoints ; i++)
        // {
        //     result.first[i].pt.x+=minBorderX;
        //     result.first[i].pt.y+=minBorderY;
        //     result.first[i].octave=0;
        //     result.first[i].size = scaledPatchSize;
        // }

        computeOrientation(image, result.first, umax);

        vector<KeyPoint>& keypoints = result.first; 
        cv::Mat descriptors_ = cv::Mat(result.first.size(), 256, CV_32FC1);
        // cv::normalize(result.second, descriptors_, 1.0, 0.0, cv::NORM_L2);

        for (int i=0; i < result.second.rows; i++) {
            cv::normalize(result.second.row(i), result.second.row(i), 1.0, 0.0, cv::NORM_L2);
            // Вычисляем L2-норму дескриптора
            // double norm = cv::norm(result.second.row(i), cv::NORM_L2);
            // Проверяем, близка ли норма к 1 (с учетом погрешности)
            // bool is_normalized = std::abs(norm - 1.0) < 1e-5;
            // std::cout << "Normalized descriptor check: " << is_normalized << std::endl; 
        }
        
        Mat descriptors;
        if( nkeypoints == 0 ) {
             _descriptors.release();
        } else {
            _descriptors.create(nkeypoints, 256, CV_32FC1);
            descriptors = _descriptors.getMat();
        }

        _keypoints = vector<cv::KeyPoint>(nkeypoints);

        //Modified for speeding up stereo fisheye matching
        int monoIndex = 0, stereoIndex = nkeypoints-1;
        float scale = mvScaleFactor[0];

        int i = 0;
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                    keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint){

            // if(keypoint->pt.x >= vLappingArea[0] && keypoint->pt.x <= vLappingArea[1]){
            //     _keypoints.at(stereoIndex) = (*keypoint);
            //     descriptors_.row(i).copyTo(descriptors.row(stereoIndex));
            //     stereoIndex--;
            // }
            // else{
            //     _keypoints.at(monoIndex) = (*keypoint);
            //     descriptors_.row(i).copyTo(descriptors.row(monoIndex));
            //     monoIndex++;
            // }
            _keypoints.at(monoIndex) = (*keypoint);
            result.second.row(i).copyTo(descriptors.row(monoIndex));
            monoIndex++;
            i++;
        }

        return monoIndex;
    }
} //namespace ORB_SLAM

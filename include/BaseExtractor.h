#ifndef BASEEXTRACTOR_H
#define BASEEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/opencv.hpp>
#include <ExtractorNode.h>

namespace ORB_SLAM3
{

class BaseExtractor
{
public:
    virtual ~BaseExtractor() = default;

    static const int PATCH_SIZE = 31;
    static const int HALF_PATCH_SIZE = 15;
    static const int EDGE_THRESHOLD = 19;

    static float IC_Angle(const cv::Mat& image, cv::Point2f pt,  const std::vector<int> & u_max)
    {
        int m_01 = 0, m_10 = 0;

        const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int)image.step1();
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v*step], val_minus = center[u - v*step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return cv::fastAtan2((float)m_01, (float)m_10);
    }

    static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const std::vector<int>& umax)
    {
        for (std::vector<cv::KeyPoint>::iterator keypoint = keypoints.begin(),
                     keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
        {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    static bool compareNodes(std::pair<int,ExtractorNode*>& e1, std::pair<int,ExtractorNode*>& e2){
        if(e1.first < e2.first){
            return true;
        }
        else if(e1.first > e2.first){
            return false;
        }
        else{
            if(e1.second->UL.x < e2.second->UL.x){
                return true;
            }
            else{
                return false;
            }
        }
    }

};

} //namespace ORB_SLAM

#endif


#ifndef BASEMATCHER_H
#define BASEMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"

// #include"ORBmatcher.h"
// #include"SuperPointMatcher.h"
// #include"SuperGlueMatcher.h"

namespace ORB_SLAM3
{

enum class eMatcherType {
    ORB,
    SuperPoint,
    SuperGlue
};

class BaseMatcher {
public:
    virtual ~BaseMatcher() = default;

    // static std::unique_ptr<BaseMatcher> create_matcher(
    //     eMatcherType type,
    //     float nn_ratio,
    //     bool check_orientation,
    //     eDescriptorDistMetric dist_metric,
    //     // const std::string& model_path = "",
    //     Ort::SuperGlue* model,
    //     const cv::Size& image_size = cv::Size())
    // {
    //     switch(type) {
    //         case eMatcherType::ORB:
    //             return std::unique_ptr<BaseMatcher>(
    //                 new ORBmatcher(nn_ratio, check_orientation, dist_metric));
                
    //         case eMatcherType::SuperPoint:
    //             return std::unique_ptr<BaseMatcher>(
    //                 new SuperPointMatcher(nn_ratio, check_orientation, dist_metric));
                
    //         case eMatcherType::SuperGlue:
    //             // if(model_path.empty()) throw std::invalid_argument("Model path required");
    //             return std::unique_ptr<BaseMatcher>(
    //                 new SuperGlueMatcher(model, image_size, nn_ratio, check_orientation, dist_metric));
                
    //         default:
    //             throw std::invalid_argument("Unknown matcher type");
    //     }
    // }

    // Computes the distance between two descriptors
    static float DescriptorDistance(const cv::Mat &a, const cv::Mat &b, const eDescriptorDistMetric distMetric = eDescriptorDistMetric::L2) {
        if (distMetric == eDescriptorDistMetric::HAMMING) {
            const int *pa = a.ptr<int32_t>();
            const int *pb = b.ptr<int32_t>();

            int dist=0;

            for(int i=0; i<8; i++, pa++, pb++)
            {
                unsigned  int v = *pa ^ *pb;
                v = v - ((v >> 1) & 0x55555555);
                v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
                dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
            }

            return static_cast<float>(dist);
        } 
        else if (distMetric == eDescriptorDistMetric::L2) {
            return cv::norm(a, b, cv::NORM_L2);
        }
    }
};

}// namespace ORB_SLAM

#endif // BASEMATCHER_H
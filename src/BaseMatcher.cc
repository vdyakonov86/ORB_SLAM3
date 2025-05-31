#include"BaseMatcher.h"
#include"ORBmatcher.h"
#include"SuperPointMatcher.h"
#include"SuperGlueMatcher.h"

std::unique_ptr<ORB_SLAM3::BaseMatcher> 
ORB_SLAM3::BaseMatcher::create_matcher(
  eMatcherType type,
  float nn_ratio,
  bool check_orientation,
  eDescriptorDistMetric dist_metric,
  Ort::SuperGlue* model,
  const cv::Size& image_size) 
{
    switch(type) {
        case eMatcherType::ORB:
            return std::make_unique<ORBmatcher>(nn_ratio, check_orientation, dist_metric);
        case eMatcherType::SUPERPOINT:
            return std::make_unique<SuperPointMatcher>(nn_ratio, check_orientation, dist_metric);
        case eMatcherType::SUPERGLUE:
            return std::make_unique<SuperGlueMatcher>(
                static_cast<Ort::SuperGlue*>(model),  // Каст обратно
                image_size, 
                nn_ratio, 
                check_orientation, 
                dist_metric
            );
        default:
            throw std::invalid_argument("Unknown matcher type");
    }
}

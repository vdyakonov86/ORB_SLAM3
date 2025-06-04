#ifndef BASEMATCHER_H
#define BASEMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include"sophus/sim3.hpp"

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"
#include <ort_utility/ort_utility.hpp>
#include <ort-superglue/SuperGlue.hpp>

namespace ORB_SLAM3
{
// Forward declarations (предварительные объявления классов)
class ORBmatcher;
class SuperPointMatcher;
class SuperGlueMatcher;

enum class eMatcherType {
    ORB,
    SUPERPOINT,
    SUPERGLUE
};

inline eMatcherType stringToMatcherType(const std::string& str) {
    if (str == "ORB") return eMatcherType::ORB;
    if (str == "SUPERPOINT") return eMatcherType::SUPERPOINT;
    if (str == "SUPERGLUE") return eMatcherType::SUPERGLUE;
    throw std::invalid_argument("Unknown matcher type: " + str);
}

class BaseMatcher {
public:
    virtual ~BaseMatcher() = default;

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    virtual int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f) = 0;

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    virtual int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono) = 0;

    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    virtual int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const float dist_high) = 0;

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    virtual int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0) = 0;

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in Place Recognition (Loop Closing and Merging)
    virtual int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0) = 0;

    // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    virtual int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches) = 0;
    virtual int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12) = 0;

    // Matching for the Map Initialization (only used in the monocular case)
    virtual int SearchForInitialization(Frame &F1, Frame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10) = 0;

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    virtual int SearchForTriangulation(KeyFrame *pKF1, KeyFrame* pKF2,
                                std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse = false) = 0;

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    // int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th) = 0;
    virtual int SearchBySim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches12, const Sophus::Sim3f &S12, const float th) = 0;

    // Project MapPoints into KeyFrame and search for duplicated MapPoints.
    virtual int Fuse(KeyFrame* pKF, const vector<MapPoint *> &vpMapPoints, const float th=3.0, const bool bRight = false) = 0;

    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    virtual int Fuse(KeyFrame* pKF, Sophus::Sim3f &Scw, const std::vector<MapPoint*> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint) = 0;
    
    static std::unique_ptr<BaseMatcher> create_matcher(
        eMatcherType type,
        float nn_ratio,
        bool check_orientation,
        eDescriptorDistMetric dist_metric,
        Ort::SuperGlue* model = nullptr,
        const cv::Size& image_size = cv::Size());

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

    static void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1=0;
        int max2=0;
        int max3=0;

        for(int i=0; i<L; i++)
        {
            const int s = histo[i].size();
            if(s>max1)
            {
                max3=max2;
                max2=max1;
                max1=s;
                ind3=ind2;
                ind2=ind1;
                ind1=i;
            }
            else if(s>max2)
            {
                max3=max2;
                max2=s;
                ind3=ind2;
                ind2=i;
            }
            else if(s>max3)
            {
                max3=s;
                ind3=i;
            }
        }

        if(max2<0.1f*(float)max1)
        {
            ind2=-1;
            ind3=-1;
        }
        else if(max3<0.1f*(float)max1)
        {
            ind3=-1;
        }
    }

    protected:
        static float RadiusByViewingCos(const float &viewCos)
        {
            if(viewCos>0.998)
                return 2.5;
            else
                return 4.0;
        }

        
};

}// namespace ORB_SLAM

#endif // BASEMATCHER_H
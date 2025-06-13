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
        mvScaleFactor.resize(nlevels);
        mvLevelSigma2.resize(nlevels);
        mvScaleFactor[0]=1.0f;
        mvLevelSigma2[0]=1.0f;
        for(int i=1; i<nlevels; i++)
        {
            mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor;
            mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(nlevels);
        mvInvLevelSigma2.resize(nlevels);
        for(int i=0; i<nlevels; i++)
        {
            mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
            mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
        }

        mvImagePyramid.resize(nlevels);

        mnFeaturesPerLevel.resize(nlevels);
        float factor = 1.0f / scaleFactor;
        float nDesiredFeaturesPerScale = nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)nlevels));

        int sumFeatures = 0;
        for( int level = 0; level < nlevels-1; level++ )
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[nlevels-1] = std::max(nfeatures - sumFeatures, 0);

        //This is for orientation
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

    // void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    // {
    //     const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    //     const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //     //Define boundaries of childs
    //     n1.UL = UL;
    //     n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    //     n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    //     n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    //     n1.vKeys.reserve(vKeys.size());

    //     n2.UL = n1.UR;
    //     n2.UR = UR;
    //     n2.BL = n1.BR;
    //     n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    //     n2.vKeys.reserve(vKeys.size());

    //     n3.UL = n1.BL;
    //     n3.UR = n1.BR;
    //     n3.BL = BL;
    //     n3.BR = cv::Point2i(n1.BR.x,BL.y);
    //     n3.vKeys.reserve(vKeys.size());

    //     n4.UL = n3.UR;
    //     n4.UR = n2.BR;
    //     n4.BL = n3.BR;
    //     n4.BR = BR;
    //     n4.vKeys.reserve(vKeys.size());

    //     //Associate points to childs
    //     for(size_t i=0;i<vKeys.size();i++)
    //     {
    //         const cv::KeyPoint &kp = vKeys[i];
    //         if(kp.pt.x<n1.UR.x)
    //         {
    //             if(kp.pt.y<n1.BR.y)
    //                 n1.vKeys.push_back(kp);
    //             else
    //                 n3.vKeys.push_back(kp);
    //         }
    //         else if(kp.pt.y<n1.BR.y)
    //             n2.vKeys.push_back(kp);
    //         else
    //             n4.vKeys.push_back(kp);
    //     }

    //     if(n1.vKeys.size()==1)
    //         n1.bNoMore = true;
    //     if(n2.vKeys.size()==1)
    //         n2.bNoMore = true;
    //     if(n3.vKeys.size()==1)
    //         n3.bNoMore = true;
    //     if(n4.vKeys.size()==1)
    //         n4.bNoMore = true;

    // }

    static bool compareNodes(pair<int,ExtractorNode*>& e1, pair<int,ExtractorNode*>& e2){
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

    void SuperPointExtractor::DistributeOctTree(
        const std::vector<cv::KeyPoint>& vToDistributeKeys, 
        const cv::Mat& vToDistributeDesc, 
        std::vector<cv::KeyPoint>& vResultKeys,
        cv::Mat& vResultDesc, 
        const int &minX, const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // Compute how many initial nodes
        const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));

        const float hX = static_cast<float>(maxX-minX)/nIni;

        list<ExtractorNode> lNodes;

        vector<ExtractorNode*> vpIniNodes;
        vpIniNodes.resize(nIni);

        for(int i=0; i<nIni; i++)
        {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
            ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
            ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
            ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            ni.vDesc.create(vToDistributeKeys.size(), 256, CV_32F);

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }
        cout << "AFTER 1st loop" << endl;
        cout << "vToDistributeKeys.size(): " << vToDistributeKeys.size() << endl;
        cout << "vToDistributeDesc.rows: " << vToDistributeDesc.rows << endl;
        //Associate points to childs
        for(size_t i=0;i<vToDistributeKeys.size();i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            const Mat &desc = vToDistributeDesc.row(i);
            vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
            cout << "vDesc.rows: " <<  vpIniNodes[kp.pt.x/hX]->vDesc.rows << endl;
            vpIniNodes[kp.pt.x/hX]->vDesc.row(i) = desc;
        }
        cout << "AFTER Associate points to childs" << endl;

        list<ExtractorNode>::iterator lit = lNodes.begin();

        while(lit!=lNodes.end())
        {
            if(lit->vKeys.size()==1)
            {
                lit->bNoMore=true;
                lit++;
            }
            else if(lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int,ExtractorNode*> > vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size()*4);

        while(!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            while(lit!=lNodes.end())
            {
                if(lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNode n1,n2,n3,n4;
                    lit->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit=lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
            {
                bFinish = true;
            }
            else if(((int)lNodes.size()+nToExpand*3)>N)
            {

                while(!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int,ExtractorNode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end(),compareNodes);
                    for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                    {
                        ExtractorNode n1,n2,n3,n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                        // Add childs if they contain points
                        if(n1.vKeys.size()>0)
                        {
                            lNodes.push_front(n1);
                            if(n1.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n2.vKeys.size()>0)
                        {
                            lNodes.push_front(n2);
                            if(n2.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n3.vKeys.size()>0)
                        {
                            lNodes.push_front(n3);
                            if(n3.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if(n4.vKeys.size()>0)
                        {
                            lNodes.push_front(n4);
                            if(n4.vKeys.size()>1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if((int)lNodes.size()>=N)
                            break;
                    }

                    if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                        bFinish = true;

                }
            }
        }

        // Retain the best point in each node
        // vector<cv::KeyPoint> vResultKeys;
        // vResultKeys.reserve(nfeatures);
        int counter = 0;
        for(list<ExtractorNode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::Mat& vNodeDesc = lit->vDesc;                   // ОК: vDesc — lvalue

            cv::KeyPoint* pKP = &vNodeKeys[0];                 // ОК: элемент вектора — lvalue
            cv::Mat rowData = vNodeDesc.row(0);                // Копируем строку
            cv::Mat* pDesc = &rowData;                         // Теперь можно взять адрес

            float maxResponse = pKP->response;

            for(size_t k=1;k<vNodeKeys.size();k++)
            {
                if(vNodeKeys[k].response>maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    rowData = vNodeDesc.row(k);                // Копируем строку
                    pDesc = &rowData;   
                    // pDesc = &vNodeDesc.row(k);
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
            vResultDesc.row(counter) = *pDesc;
            counter++;
        }
        cout << "AFTER Retain the best point in each node" << endl;
    }

    void SuperPointExtractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints, vector<Mat>& allDescriptors)
    {
        allKeypoints.resize(nlevels);
        allDescriptors.resize(nlevels);

        const float W = 35;

        for (int level = 0; level < nlevels; ++level)
        {
            const int minBorderX = EDGE_THRESHOLD-3;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
            const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

            vector<cv::KeyPoint> vToDistributeKeys;
            Mat vToDistributeDesc;
            vToDistributeKeys.reserve(nfeatures*10);
            vToDistributeDesc.create(nfeatures*10, 256, CV_32F);

            const float width = (maxBorderX-minBorderX);
            const float height = (maxBorderY-minBorderY);

            const int nCols = width/W;
            const int nRows = height/W;
            const int wCell = ceil(width/nCols);
            const int hCell = ceil(height/nRows);

            cout << "BEFORE inference" << endl;
            int counter = 0;
            for(int i=0; i<nRows; i++)
            {
                const float iniY =minBorderY+i*hCell;
                float maxY = iniY+hCell+6;

                if(iniY>=maxBorderY-3)
                    continue;
                if(maxY>maxBorderY)
                    maxY = maxBorderY;

                for(int j=0; j<nCols; j++)
                {
                    const float iniX =minBorderX+j*wCell;
                    float maxX = iniX+wCell+6;
                    if(iniX>=maxBorderX-6)
                        continue;
                    if(maxX>maxBorderX)
                        maxX = maxBorderX;

                    vector<cv::KeyPoint> vKeysCell;
                    cout << "iniY: " << iniY << " maxY: " << maxY << " iniX: " << iniX << " maxX: " << maxX << endl;
                    auto img = mvImagePyramid[level].rowRange(iniY,maxY).colRange(iniX,maxX);
                    cout << "inference" << endl;
                    KeyPointAndDesc result = superPoint->inference(*superPoint, img, 4, 0.015, true, 2);

                    cout << "inference end" << endl;
                    vKeysCell = result.first;
                    cout << "result.second.cols " << result.second.cols << endl;

                    if(!vKeysCell.empty())
                    {
                        for(vector<cv::KeyPoint>::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                        {
                            (*vit).pt.x+=j*wCell;
                            (*vit).pt.y+=i*hCell;
                            vToDistributeKeys.push_back(*vit);
                        }
                        for (int i=0; i < result.second.rows; i++) {
                            cv::normalize(result.second.row(i), result.second.row(i), 1.0, 0.0, cv::NORM_L2);
                            vToDistributeDesc.row(counter) = result.second.row(i);
                            cout << "counter: " << counter << " nfeatures*10: " << nfeatures*10 << endl;
                            counter++;
                        }
                    
                        // vToDistributeDesc = result.second;
                    }

                }
            }
            cout << "AFTER inference" << endl;

            vector<KeyPoint> & keypoints = allKeypoints[level];
            Mat & descriptors = allDescriptors[level];
            cout << "LEVEL: " << level << endl;
            keypoints.reserve(nfeatures);
            descriptors.create(nfeatures, 256, CV_32F);

            cout << "BEFORE DistributeOctTree" << endl;

            DistributeOctTree(
                vToDistributeKeys, 
                vToDistributeDesc, 
                keypoints,
                descriptors, 
                minBorderX, maxBorderX, minBorderY, maxBorderY,mnFeaturesPerLevel[level], level);

            const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for(int i=0; i<nkps ; i++)
            {
                keypoints[i].pt.x+=minBorderX;
                keypoints[i].pt.y+=minBorderY;
                keypoints[i].octave=level;
                keypoints[i].size = scaledPatchSize;
            }
        }

        // compute orientations
        for (int level = 0; level < nlevels; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    // void SuperPointExtractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint> >& allKeypoints, vector<Mat>& allDescriptors)
    // {
    //     allKeypoints.resize(nlevels);
    //     allDescriptors.resize(nlevels);

    //     for (int level = 0; level < nlevels; ++level)
    //     {
    //         const int minBorderX = EDGE_THRESHOLD-3;
    //         const int minBorderY = minBorderX;
    //         const int maxBorderX = mvImagePyramid[level].cols-EDGE_THRESHOLD+3;
    //         const int maxBorderY = mvImagePyramid[level].rows-EDGE_THRESHOLD+3;

    //         KeyPointAndDesc result = superPoint->inference(*superPoint, mvImagePyramid[level], 4, 0.015, true, 2);
    //         allKeypoints[level] = result.first;

    //         for (int i=0; i < result.second.rows; i++) {
    //             cv::normalize(result.second.row(i), result.second.row(i), 1.0, 0.0, cv::NORM_L2);
    //         }
    //         allDescriptors[level] = result.second;

    //         const int scaledPatchSize = PATCH_SIZE*mvScaleFactor[level];

    //         // Add border to coordinates and scale information
    //         const int nkps = allKeypoints[level].size();
    //         for(int i=0; i<nkps ; i++)
    //         {
    //             allKeypoints[level][i].pt.x+=minBorderX;
    //             allKeypoints[level][i].pt.y+=minBorderY;
    //             allKeypoints[level][i].octave=level;
    //             allKeypoints[level][i].size = scaledPatchSize;
    //         }
    //     }

    //     // compute orientations
    //     for (int level = 0; level < nlevels; ++level)
    //         computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    // }

    int SuperPointExtractor::operator()( InputArray _image, InputArray _mask, vector<KeyPoint>& _keypoints,
                                  OutputArray _descriptors, std::vector<int> &vLappingArea)
    {
        //cout << "[SuperPointExtractor]: Max Features: " << nfeatures << endl;
        if(_image.empty())
            return -1;

        Mat image = _image.getMat();
        assert(image.type() == CV_8UC1 );

        // Pre-compute the scale pyramid
        ComputePyramid(image);

        vector<vector<KeyPoint>> allKeypoints;
        vector<Mat> allDescriptors;
        cout << "ComputeKeyPointsOctTree START: " << endl;
        ComputeKeyPointsOctTree(allKeypoints, allDescriptors);
        cout << "ComputeKeyPointsOctTree END: " << endl;
        Mat descriptors;

        int nkeypoints = 0;
        for (int level = 0; level < nlevels; ++level)
            nkeypoints += (int)allKeypoints[level].size();
        if( nkeypoints == 0 )
            _descriptors.release();
        else
        {
            _descriptors.create(nkeypoints, 256, CV_32F);
            descriptors = _descriptors.getMat();
        }

        _keypoints = vector<cv::KeyPoint>(nkeypoints);

        int offset = 0;
        //Modified for speeding up stereo fisheye matching
        int monoIndex = 0, stereoIndex = nkeypoints-1;
        for (int level = 0; level < nlevels; ++level)
        {
            vector<KeyPoint>& keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            if(nkeypointsLevel==0)
                continue;

            // preprocess the resized image
            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors
            //Mat desc = descriptors.rowRange(offset, offset + nkeypointsLevel);
            // Mat desc = cv::Mat(nkeypointsLevel, 32, CV_8U);
            Mat& desc = allDescriptors[level];

            offset += nkeypointsLevel;


            float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
            int i = 0;
            for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint){

                // Scale keypoint coordinates
                if (level != 0){
                    keypoint->pt *= scale;
                }

                if(keypoint->pt.x >= vLappingArea[0] && keypoint->pt.x <= vLappingArea[1]){
                    _keypoints.at(stereoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(stereoIndex));
                    stereoIndex--;
                }
                else{
                    _keypoints.at(monoIndex) = (*keypoint);
                    desc.row(i).copyTo(descriptors.row(monoIndex));
                    monoIndex++;
                }
                i++;
            }
        }
        return monoIndex;
    }

    void SuperPointExtractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < nlevels; ++level)
        {
            float scale = mvInvScaleFactor[level];
            Size sz(cvRound((float)image.cols*scale), cvRound((float)image.rows*scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD*2, sz.height + EDGE_THRESHOLD*2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if( level != 0 )
            {
                resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);

                copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101+BORDER_ISOLATED);
            }
            else
            {
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101);
            }
        }

    }

} //namespace ORB_SLAM

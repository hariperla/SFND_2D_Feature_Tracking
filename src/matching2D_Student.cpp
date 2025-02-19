#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorDistType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        if (descriptorDistType.compare("DES_BINARY") == 0) 
        {
            matcher = cv::BFMatcher::create(cv::NORM_HAMMING, crossCheck);
        } else if (descriptorDistType.compare("DES_HOG") == 0) 
        {
            matcher = cv::BFMatcher::create(cv::NORM_L2, crossCheck);
        }
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        // We do this conversion to avoid errors based on current openCV implementation of FLANN method
        if (descSource.type() != CV_32F || descRef.type() != CV_32F)
        {
            descSource.convertTo(descSource,CV_32F);
            descRef.convertTo(descRef,CV_32F);
        }
        /* Create FLANN based descriptor */
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        // ...
        vector<vector<cv::DMatch>> knn_matches;
        /* Find 2 nearest neighbors using knn search */
        matcher->knnMatch(descSource,descRef,knn_matches,2);
        /* Filter matches based on dist ratio test */
        float minDistRatio = 0.8;
        for (int i = 0; i < knn_matches.size(); i++)
        {
            /* code */
            float distRatio = (knn_matches.at(i).at(0).distance)/(knn_matches.at(i).at(1).distance);

            if (distRatio < minDistRatio)
            {
                matches.push_back(knn_matches.at(i).at(0)); // Filter all good matches
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
        cout << "knn points size = " << knn_matches.size() << endl;  
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
double descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        /* bool orientationNormalized = true;
        bool scaleNormalized = true;
        float patternScale = 22.0f;
        int nOctaves = 4; */
        //extractor = cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves);
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        /* int nfeatures = 0;
        int nOctaveLayers = 3;
        double contrastThreshold = 0.04;
        double edgeThreshold = 10; 
        double sigma = 1.6; */
        //extractor = cv::SiftDescriptorExtractor::create(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
        extractor = cv::SiftDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        /* int nfeatures = 500;
        float scaleFactor = 1.2f;
        int nlevels = 8;
        int edgeThreshold = 31;
        int firstLevel = 0;
        int WTA_K = 2;
        cv::ORB::ScoreType scoreType = cv::ORB::HARRIS_SCORE;
        int patchSize = 31;
        int fastThreshold = 20; */
        //extractor = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel,
        //    WTA_K, scoreType, patchSize, fastThreshold);
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        /* cv::AKAZE::DescriptorType descriptor_type = cv::AKAZE::DESCRIPTOR_MLDB;
        int descriptor_size = 0;
        int descriptor_channels = 3;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;
        cv::KAZE::DiffusivityType diffusivity = cv::KAZE::DIFF_PM_G2; */
        //extractor = cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, 
        //    threshold, nOctaves, nOctaveLayers, diffusivity);
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        /* int bytes=32;
        bool use_orientation = false; */
        //extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(bytes,use_orientation);
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else
    {
        //... Do nothing
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;

    return 1000 * t / 1.0;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return 1000 * t / 1.0;
}

/* Detect key points using Harris corner detector */
double detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // Look for prominent corners and instantiate keypoints
    float maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {
            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            { // only store points above a threshold
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = response;
                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                for (int k = 0; k < keypoints.size(); k++)
                {
                    /* code */
                    float keyPtOverlap = cv::KeyPoint::overlap(newKeyPoint,keypoints.at(k));
                    if (keyPtOverlap > maxOverlap)
                    {
                        /* code */
                        bOverlap = true;
                        if (newKeyPoint.response > keypoints.at(k).response)
                        {
                            /* code */
                            keypoints.at(k) = newKeyPoint;
                            break;
                        }   
                    }  
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        } 
    }  

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 7);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }   

    return 1000 * t / 1.0;
}

double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    // select appropriate feature detector
    cv::Ptr<cv::FeatureDetector> featDetector;

    if (detectorType.compare("SIFT") == 0)
    {
        featDetector = cv::SIFT::create();
    }
    else if (detectorType.compare("FAST") == 0)
    {
        int fastThrsh = 10;
        featDetector = cv::FastFeatureDetector::create(fastThrsh,true,cv::FastFeatureDetector::TYPE_9_16);
    }
    else if (detectorType.compare("ORB") == 0)
    {
        featDetector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        featDetector = cv::AKAZE::create();
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        featDetector = cv::BRISK::create();
    }
    else
    {
        // Do nothing
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    featDetector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << "# of key points " << keypoints.size() << " detected in " << 1000 * t / 1.0 << " ms" << endl;

    return 1000 * t / 1.0;
}
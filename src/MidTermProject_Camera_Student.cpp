/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <numeric>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <filesystem>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* Create two vectors for detector types and descriptor types */
    vector<string> detTypes = { "SHITOMASI", "HARRIS", "FAST", "SIFT", "ORB", "AKAZE", "BRISK"};
    //vector<string> descTypes = {"SIFT", "ORB", "BRIEF", "FREAK", "AKAZE","BRISK"};
    vector<string> descTypes = {"SIFT"};

    vector <int> saveKeyPoints;
    vector <vector<int>> keyPtsWithDetectors;
    vector <vector<int>> matchesVec;
    vector <double> runTime;
    vector <double> avgRunTime;
    vector <int> saveMatchedPoints;
    vector<vector<double>> runTimeAllImages;

    /* Perform metrics for all different detector types and descriptor types */
    for (string detectorType : detTypes)
    {
        cout << "detector Type: " << detectorType << endl;
        for (string descriptorType : descTypes)
        {
            cout << "descriptor Type: " << descriptorType << endl;
            if (
                (detectorType == "SIFT" && descriptorType == "ORB") ||
                (detectorType == "SIFT" && descriptorType == "AKAZE") ||
                (detectorType == "ORB" && descriptorType == "AKAZE") ||
                (detectorType == "HARRIS" && descriptorType == "AKAZE") ||
                (detectorType == "SHITOMASI" && descriptorType == "AKAZE") ||
                (detectorType == "FAST" && descriptorType == "AKAZE") ||
                (detectorType == "BRISK" && descriptorType == "AKAZE")
               )
            {
                continue;
            }
            /* MAIN LOOP OVER ALL IMAGES */
            for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
            {
                /* LOAD IMAGE INTO BUFFER */

                // assemble filenames for current index
                ostringstream imgNumber;
                imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
                string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

                cout << "fileName: " << imgFullFilename << endl; 

                // load image from file and convert to grayscale
                cv::Mat img, imgGray;
                img = cv::imread(imgFullFilename);
                cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

                //// STUDENT ASSIGNMENT
                //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

                // push image into data frame buffer
                DataFrame frame;
                frame.cameraImg = imgGray;
                if (dataBuffer.size() > dataBufferSize)
                {
                    dataBuffer.erase(dataBuffer.begin()); // Erase the first element
                }
                dataBuffer.push_back(frame); // Push the element into the back of the frame

                //// EOF STUDENT ASSIGNMENT
                cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                vector<cv::KeyPoint> keypoints; // create empty feature list for current image
                //string detectorType = "ORB";
                double detectorRunTime;

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

                if (detectorType.compare("SHITOMASI") == 0)
                {
                   detectorRunTime = detKeypointsShiTomasi(keypoints, imgGray, false);
                }
                else if (detectorType.compare("HARRIS") == 0)
                {
                    detectorRunTime = detKeypointsHarris(keypoints, imgGray, false);
                }
                else
                {
                    //...
                    detectorRunTime = detKeypointsModern(keypoints, imgGray, detectorType, false);
                }
                //// EOF STUDENT ASSIGNMENT

                //// STUDENT ASSIGNMENT
                //// TASK MP.3 -> only keep keypoints on the preceding vehicle

                // only keep keypoints on the preceding vehicle
                bool bFocusOnVehicle = true;
                cv::Rect vehicleRect(535, 180, 180, 150);
                vector<cv::KeyPoint> kpInRect; // Key points in the rectangle
                if (bFocusOnVehicle)
                {
                    // ...  
                    for (int i = 0; i < keypoints.size(); i++)
                    {
                        if (vehicleRect.contains(keypoints.at(i).pt))
                        {
                            kpInRect.push_back(keypoints.at(i));
                        }
                    } 
                    keypoints = kpInRect; // Update keypoints to the points only in the rectangle    
                    cout << "key points in rectangle = " << keypoints.size() << endl;    
                }

                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                bool bLimitKpts = false;
                if (bLimitKpts)
                {
                    int maxKeypoints = 50;

                    if (detectorType.compare("SHITOMASI") == 0)
                    { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    cout << " NOTE: Keypoints have been limited!" << endl;
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;
                cout << "#2 : DETECT KEYPOINTS done" << endl;
                saveKeyPoints.push_back(keypoints.size());

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;
                //string descriptorType = "ORB"; // BRIEF, ORB, FREAK, AKAZE, SIFT
                double descriptorRunTime;
                descriptorRunTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
                //// EOF STUDENT ASSIGNMENT
                //cout << "descriptor run time: " << descriptorRunTime << endl;
                runTime.push_back((detectorRunTime + descriptorRunTime));

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {
                    /* MATCH KEYPOINT DESCRIPTORS */

                    vector<cv::DMatch> matches;
                    string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    string descriptorDistType = "DES_BINARY"; // DES_BINARY, DES_HOG
                    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    /* Since SIFT and AKAZE are non binary descriptors, we cannot use Hammond distance to calculate matches */
                    if (descriptorType.compare("SIFT") || descriptorType.compare("AKAZE"))
                    {
                        descriptorDistType = "DES_HOG";
                    }

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                    (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                    matches, descriptorDistType, matcherType, selectorType);

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;
                    saveMatchedPoints.push_back(matches.size());

                    cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

                    // visualize matches between current and previous image
                    bVis = true;
                    if (bVis)
                    {
                        cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                        cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                        (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                        matches, matchImg,
                                        cv::Scalar::all(-1), cv::Scalar::all(-1),
                                        vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                        string windowName = "Matching keypoints between two camera images";
                        cv::namedWindow(windowName, 7);
                        cv::imshow(windowName, matchImg);
                        cout << "Press key to continue to next image" << endl;
                        cv::waitKey(100); // wait for key to be pressed
                    }
                    bVis = false;
                }
            } // eof loop over all images 
        }
        keyPtsWithDetectors.push_back(saveKeyPoints);
        saveKeyPoints.clear();  // Clear after each detector, already existing keypoints as we do not want to keep adding
        matchesVec.push_back(saveMatchedPoints);
        saveMatchedPoints.clear();// Clear after each detector, already existing keypoints as we do not want to keep adding 

        // Compute avg run time for all 10 images for a particular detector - descriptor type
        double tot_time = 0.0;
        for (size_t p = 0; p < runTime.size(); p++)
        {
            tot_time += runTime.at(p);
        }
        double avg_time = tot_time/(runTime.size() * 1.0);
        avgRunTime.push_back(avg_time);
        
        runTimeAllImages.push_back(runTime);
        runTime.clear();
    }

    /* Store the final timing results in a csv */
    ofstream detDescRunTime;
    detDescRunTime.open("../SFND_2d_cam_midterm_task9_PerformanceMetrics.csv", ios::app);

    /* Loop through the detector and descriptor types and print the avg run times for each combination */
    for (size_t r = 0; r < avgRunTime.size(); r++)
    {
        detDescRunTime << detTypes.at(r) << "-" << descTypes.at(0) << "," << avgRunTime.at(r) << endl;
        //cout << "Avg Run Time for " << detTypes.at(r) << "-" << descTypes.at(0) << ":" << fixed << avgRunTime.at(r) << endl;
    }
    detDescRunTime << endl;
    detDescRunTime.close();   

    /* Write the results to a csv */
    /* ofstream keyPointsCSV;
    keyPointsCSV.open("../SFND_2d_cam_midterm_task7_KeyPointsDistribution.csv", ios::app);

    for (size_t rows = 0; rows < keyPtsWithDetectors.size(); rows++)
    {
        keyPointsCSV << detTypes.at(rows);
        for (size_t cols = 0; cols < keyPtsWithDetectors[rows].size(); cols++)
        {
            keyPointsCSV << "," << keyPtsWithDetectors.at(rows).at(cols);
        }
        keyPointsCSV << endl;
    }
    keyPointsCSV << endl; */

    /* Store matched keypoint results in a file */
    ofstream matchedKeyPtsFile;
    matchedKeyPtsFile.open("../SFND_2d_cam_midterm_task8_MatchedKeyPoints.csv",ios::app);
    for (size_t rows = 0; rows < matchesVec.size(); rows++)
    {
        matchedKeyPtsFile << detTypes.at(rows) << "-" << descTypes.at(0);
        for (size_t cols = 0; cols < matchesVec[rows].size(); cols++)
        {
            matchedKeyPtsFile << "," << matchesVec.at(rows).at(cols);
        }
        matchedKeyPtsFile << endl;
    }
    matchedKeyPtsFile << endl;
    matchedKeyPtsFile.close();

    return 0;
}
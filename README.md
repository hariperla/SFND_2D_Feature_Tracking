# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Build instructions and Dependency
* Original build and dependency instructions are found below
```
https://github.com/udacity/SFND_2D_Feature_Tracking.git
```

## Note: 
```
Modified return type for detector and descriptor functions to return time it takes to run. 
```

#### Task 1 - Create and use a ring buffer
```c++
// push image into data frame buffer
DataFrame frame;
frame.cameraImg = imgGray;
if (dataBuffer.size() > dataBufferSize)
{
  dataBuffer.erase(dataBuffer.begin()); // Erase the first element
}
dataBuffer.push_back(frame); // Push the element into the back of the frame
```

#### Task 2 - Detect keypoints using different methods
```c++
//// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT,SHI-TOMASI
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
```

#### Task 3 - Fit a box on proceeding vehicle
```c++
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
```

#### Task 4 - Extracting descriptors from the key points
```c++
//// -> BRIEF, ORB, FREAK, AKAZE, SIFT, BRISK
cv::Mat descriptors;
double descriptorRunTime;
descriptorRunTime = descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
```

#### Task 5,6 - Add FLANN matching and kNN matching with threshold of 0.8 to filter good matches
```c++
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

else if (selectorType.compare("SEL_KNN") == 0)
{ // k nearest neighbors (k=2)
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
}
```

#### Task 7 - Performance evaluation of keypoints for different detectors
> Detecting key points using different methods. 
> 
> Below is a markdown indicating all the keypoints found for 10 images using different detectors
> 
| # |DETECTOR |0  |1  |2  |3  |4  |5  |6  |7  |8  |9  |
|---|---------|---|---|---|---|---|---|---|---|---|---|
|1  |SHITOMASI|125|118|123|120|120|113|114|123|111|112|
|2  |HARRIS   |17 |14 |18 |21 |26 |43 |18 |31 |26 |34 |
|3  |FAST     |419|427|404|423|386|414|418|406|396|401|
|4  |SIFT     |138|132|124|137|134|140|137|148|159|137|
|5  |ORB      |92 |102|106|113|109|125|130|129|127|128|
|6  |AKAZE    |166|157|161|155|163|164|173|175|177|179|
|7  |BRISK    |264|282|282|277|297|279|289|272|266|254|

#### Task 8 - Performance evaluation of matched keypoints using different detector and descriptor combinations
> Compare matched keypoints for different DETECTOR-DESCRIPTOR combinations
>
> Combinations are DETECTORS = { "SHITOMASI", "HARRIS", "FAST", "SIFT", "ORB", "AKAZE", "BRISK"}
>
> DESCRIPTORS  = {"SIFT", "ORB", "BRIEF", "FREAK", "AKAZE","BRISK"}
>
|DETECTOR-DESCRIPTOR|0  |1  |2  |3  |4  |5  |6  |7  |8  |9  |
|-------------------|---|---|---|---|---|---|---|---|---|---|
|SHITOMASI-SIFT     |112|109|104|103|99 |101|96 |106|97 |100|
|HARRIS-SIFT        |13 |14 |11 |16 |19 |22 |22 |13 |24 |22 |
|FAST-SIFT          |4  |316|325|297|311|291|326|315|300|301|
|SIFT-SIFT          |5  |82 |81 |85 |93 |90 |81 |82 |102|104|
|ORB-SIFT           |3  |67 |79 |78 |79 |82 |95 |95 |94 |94 |
|AKAZE-SIFT         |0  |134|134|130|136|137|147|147|154|151|
|BRISK-SIFT         |2  |182|193|169|183|171|195|194|176|183|
|SHITOMASI-ORB      |87 |82 |87 |90 |83 |77 |84 |86 |89 |84 |
|HARRIS-ORB         |26 |11 |11 |15 |17 |20 |18 |12 |21 |20 |
|FAST-ORB           |7  |218|229|219|222|224|223|246|234|234|
|ORB-ORB            |11 |40 |52 |46 |54 |53 |65 |68 |65 |72 |
|AKAZE-ORB          |3  |102|91 |97 |86 |95 |114|107|112|118|
|BRISK-ORB          |2  |97 |103|91 |92 |82 |117|109|108|114|
|SHITOMASI-BRIEF    |96 |93 |92 |89 |92 |93 |85 |91 |85 |90 |
|HARRIS-BRIEF       |29 |12 |12 |14 |17 |17 |16 |12 |20 |21 |
|FAST-BRIEF         |9  |229|253|233|247|224|243|251|260|238|
|SIFT-BRIEF         |65 |63 |72 |64 |66 |52 |57 |72 |67 |84 |
|ORB-BRIEF          |8  |37 |38 |37 |53 |42 |64 |58 |62 |59 |
|AKAZE-BRIEF        |31 |108|116|110|109|116|129|133|135|131|
|BRISK-BRIEF        |26 |138|166|129|141|148|155|158|161|148|
|SHITOMASI-FREAK    |68 |69 |64 |64 |63 |61 |62 |62 |62 |64 |
|HARRIS-FREAK       |15 |11 |9  |12 |14 |12 |18 |11 |16 |17 |
|FAST-FREAK         |3  |175|193|157|174|156|176|193|171|175|
|SIFT-FREAK         |14 |58 |63 |53 |63 |50 |50 |46 |52 |65 |
|ORB-FREAK          |15 |39 |33 |38 |41 |33 |40 |42 |39 |44 |
|AKAZE-FREAK        |0  |104|104|92 |101|97 |111|124|117|116|
|BRISK-FREAK        |8  |112|123|109|118|102|127|135|133|131|
|AKAZE-AKAZE        |128|128|125|117|121|132|137|140|144|133|
|SHITOMASI-BRISK    |84 |80 |73 |77 |74 |70 |79 |81 |72 |75 |
|HARRIS-BRISK       |12 |11 |9  |10 |11 |16 |14 |12 |21 |17 |
|FAST-BRISK         |5  |213|216|187|205|185|200|215|203|208|
|SIFT-BRISK         |13 |57 |63 |58 |61 |55 |52 |54 |63 |73 |
|ORB-BRISK          |2  |60 |65 |65 |76 |72 |83 |83 |73 |72 |
|AKAZE-BRISK        |0  |126|112|121|117|114|119|134|140|127|
|BRISK-BRISK        |3  |138|144|133|144|139|155|137|150|158|

#### Task 9 - Performance evaluation of run time for using different detector and descriptor combinations
> Mark down of all combinations of different detector and descriptor types for average run time across all 10 images
>
|DETECTOR-DESCRIPTOR|TIME   |
|-------------------|-------|
|FAST-BRIEF         |3.70788|
|FAST-ORB           |4.23225|
|FAST-BRISK         |4.85147|
|SHITOMASI-BRIEF    |6.313  |
|SHITOMASI-BRISK    |6.74477|
|SHITOMASI-ORB      |7.98298|
|HARRIS-BRISK       |8.89113|
|ORB-BRIEF          |9.1646 |
|ORB-BRISK          |9.27767|
|HARRIS-BRIEF       |10.4713|
|HARRIS-ORB         |10.9616|
|FAST-SIFT          |13.2345|
|SHITOMASI-SIFT     |13.9942|
|ORB-ORB            |14.1566|
|FAST-FREAK         |15.7005|
|SHITOMASI-FREAK    |16.0335|
|HARRIS-FREAK       |16.0937|
|HARRIS-SIFT        |16.9065|
|ORB-FREAK          |18.4961|
|ORB-SIFT           |22.9808|
|BRISK-BRIEF        |43.3236|
|BRISK-BRISK        |44.3299|
|AKAZE-BRIEF        |44.4428|
|AKAZE-ORB          |44.5521|
|SIFT-BRISK         |44.8536|
|SIFT-BRIEF         |45.4576|
|AKAZE-BRISK        |45.7997|
|BRISK-ORB          |45.8977|
|AKAZE-FREAK        |50.609 |
|BRISK-FREAK        |50.6434|
|SIFT-FREAK         |51.6338|
|AKAZE-SIFT         |53.3975|
|BRISK-SIFT         |54.3152|
|AKAZE-AKAZE        |74.024 |
|SIFT-SIFT          |75.5009|
|SIFT-ORB           |nan    |
|SHITOMASI-AKAZE    |nan    |
|HARRIS-AKAZE       |nan    |
|FAST-AKAZE         |nan    |
|SIFT-AKAZE         |nan    |
|ORB-AKAZE          |nan    |
|BRISK-AKAZE        |nan    |

> This concludes the analysis and gives us our top 3 performers as
>
|DETECTOR-DESCRIPTOR|TIME   |
|-------------------|-------|
|FAST-BRIEF         |3.70788|
|FAST-ORB           |4.23225|
|FAST-BRISK         |4.85147|
>
>

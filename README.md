# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

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


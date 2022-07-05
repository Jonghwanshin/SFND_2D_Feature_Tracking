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

## Writeup

I have implemented feature tracking using various keypoint detectors and descriptors.
Then, I compared each techniques for its performance and speed to get the most appropriate approach for given test images. 
I described how I accomplished all given tasks below.

### For readers
I changed the code structure in `MidTermProject_Camera_Student.cpp` to remove redundant code. I moved the whole logic to the `evalImages()` function to reuse the code for performance evaluation.
Please refer `evalImages()` function to see all project rubics are achieved.

### MP.1 Databuffer implementation

I implemented a ring buffer using `std::deque` since it has `O(1)` time complexity when inserting and removing element from each ends. 
Please refer `class RingBuffer` on `dataStructures.h` for the details.
I made some changes on `MidTermProject_Camera_Student.cpp` to work with the new data structure.

### MP.2 Keypoint Detection

I implemented keypoint detectors in file `matching2D_Students.cpp`.
For Harris detector, I implemented a non-maximum suppression(NMS) algorithms for detected keypoints in `detKeypointsHarris()` function.
For FAST, BRISK, ORB, AKAZE, SIFT detectors, I implemented each detectors in `detKeypointsModern()` function.

### MP.3 Keypoint Removal

I used a OpenCV builtin function `contains()` for this task.
Please refer L131-L145 in `MidTermProject_Camera_Student.cpp`.

### MP.4 Keypoint Descriptors

I implemented BRISK, BRIEF, FREAK, ORB, AKAZE, SIFT keypoint descriptors.
Please refer `descKeypoints()` function in `matching2D_Students.cpp`.

### MP.5 Descriptor Matching

I implemented FLANN based matcher for matching descriptors in two consecutive images.
Note that SIFT desciptor is a histogram of gradiant(HoG) descriptor therefore I added a condition in L202-203 in `MidTermProject_Camera_Student.cpp`.
Please refer `matchDescriptors()` function in `matching2D_Students.cpp`.

### MP.6 Descriptor Distance Ratio

I implemented k-NN match using `knnMatch()` function in OpenCV library.
I tested some numbers for get the best `minDiscDistRatio` and `0.8` seems to the best choice.


I implemented a function(`evalPerformance()`) for performance evaluation for all available detectors and descriptors. The function evaluates all images and measures the execution time and how many keypoints are found and matched. Then, I created a pivot table based on evaluation result.
See the below table for evaluation result.

![Performance Evaluation](images/PerformanceEvaluation.png)
### MP.7 Performance Evaluation 1

I counted the number of keypoints found and neighborhood size of all available detectors in `result.csv`. The above table also shows the summarized results of this task. Based on the average and standard deviation of each detector, I could determine the best three detectors in below:

1. FAST 
  - Keypoints Detected: Avg: 409.4, StdDev: 12.997
  - Neighbothood Size: Avg: 7.0, StdDev: 0.0 (it's fixed!)
2. BRISK
  - Keypoints Detected: Avg: 276.2, StdDev: 12.630
  - Neighbothood Size: Avg: 21.942, StdDev: 0.718 
3. AKAZE
  - Keypoints Detected: Avg: 167.0, StdDev: 8.144
  - Neighbothood Size: Avg: 7.693, StdDev: 0.145

### MP.8 Performance Evaluation 2

I counted the number of matched descriptors of all combinations of available detectors and descriptors in `result.csv`. The above table also shows the summarized results of this task. Based on the average and standard deviation of each matches, I could determine the best three descriptors with FAST detectors in below:

1. BRIEF (Avg Matches: 314.556, StdDev: 18.146)
2. SIFT (Avg Matches: 309.111, StdDev: 12.484)
3. ORB (Avg Matches: 306.889, StdDev: 11.911)
### MP.9 Performance Evaluation 3

I measured execution time for all detectors and descriptors on all given images and selected best feature tracker based on how many keypoints are found/matched and execution time.
The top 3 detector/descriptor combination as below:

1. FAST-BRIEF: Avg Keypoints found - 409.4, Avg Keypoints matched - 314.556, Avg Execution Time: 1.337 + 0.522 = 1.859ms 
2. FAST-ORB: Avg Keypoints found - 409.4, Avg Keypoints matched - 309.556, Avg Execution Time: 1.337 + 0.825 = 2.162ms 
3. FAST-SIFT: Avg Keypoints found - 409.4, Avg Keypoints matched - 306.889, Avg Execution Time: 1.337 + 8.612 = 9.949ms 

* Note that the execution time of detector is assumed to have lowest average detection time since all three detectors are same.

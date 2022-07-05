/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

/**
 * @brief this function tracks 2D feature on given images.
 * 
 * @param detectorType string, the type of detector
 * @param descriptorType string, the type of descriptor
 * @param bVis bool, a flag for visualization
 * @param bSaveResult bool, a flag for save result to the file
 * @return int 
 */
int evalImages(string detectorType="SHITOMASI", string descriptorType="ORB", bool bVis=false, bool bSaveResult=true);

/**
 * @brief evaluate performance for all available detectors and descriptors for TASK MP.5 through MP.9
 * 
 * @return int 
 */
int evalPerformance();

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* demonstrate for best feature tracking method */
    evalImages("FAST", "BRIEF", true, false);

    /* evaluate performance */
    evalPerformance();

    return 0;
}

int evalImages(string detectorType, string descriptorType, bool bVis, bool bSaveResults) 
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
    RingBuffer<DataFrame> dataBuffer = RingBuffer<DataFrame>(dataBufferSize); // list of data frames which are held in memory at the same time
    ofstream outFile("../result.csv", ios::app);
    stringstream stream;
    stream.str("");

    /* MAIN LOOP OVER ALL IMAGES */
    cout << "Evaluation with " << detectorType << " detector and " << descriptorType << " descriptor" << endl;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */
        stream << "\n" << detectorType << "," << descriptorType << "," << imgIndex << ",";

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push(frame);
        //// EOF STUDENT ASSIGNMENT
        if(bVis)
            cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
        // extract 2D keypoints from current image
        double t = (double)cv::getTickCount();
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if(detectorType.compare("HARRIS") == 0) 
        {
            detKeypointsHarris(keypoints, imgGray, false);
        }
        else
        {
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        if(bSaveResults)
            stream << 1000 * t / 1.0 << ",";
        //// EOF STUDENT ASSIGNMENT
        
        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle
        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        double total = 0.0f;
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> kp_temp;
            //first sort keypoints in x direction
            for(int i = 0; i < keypoints.size(); ++i)
            {
                if(vehicleRect.contains(keypoints[i].pt)) 
                {
                    kp_temp.push_back(keypoints[i]);
                    total += (keypoints[i].size);
                }
            }
            keypoints.clear();
            keypoints.assign(kp_temp.begin(), kp_temp.end());
        }
        if(bSaveResults) 
        {
            stream << keypoints.size() << ",";
            stream << total / keypoints.size() << ",";
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
        //measure time
        if(bVis)
            cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
        cv::Mat descriptors;
        t = (double)cv::getTickCount();
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        if(bSaveResults)
            stream << 1000 * t / 1.0 << ",";
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        if(bVis)
            cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */
            vector<cv::DMatch> matches;
            
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorClass = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN

            if(descriptorType.compare("SIFT") == 0)
                descriptorClass = "DES_HOG";

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp
            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorClass, matcherType, selectorType);
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;
            if(bSaveResults)
                stream << matches.size() << ",";

            if(bVis)
                cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
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
                cv::waitKey(0); // wait for key to be pressed
            }
        } 

    } // eof loop over all images
    if(bSaveResults)
    {
        outFile << stream.rdbuf();// write result to the file    
    }
    outFile.close();
    return 0;
}

int evalPerformance()
{
    vector<string> detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    vector<string> descriptorTypes = {"BRIEF", "BRISK", "ORB", "FREAK", "SIFT"};

    for(auto detectorType: detectorTypes)
    {
        for(auto descriptorType: descriptorTypes)
        {
            /* ORB descriptor doesn't work with SIFT detector */
            if(detectorType.compare("SIFT") == 0 && descriptorType.compare("ORB") == 0)
                continue;
            evalImages(detectorType, descriptorType);        
        }
    }
    /* AKAZE Descriptor only works with AKAZE Detector */
    evalImages("AKAZE", "AKAZE");
    return 0;
}
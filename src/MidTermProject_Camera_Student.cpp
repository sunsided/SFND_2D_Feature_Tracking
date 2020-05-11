#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>

#include "dataStructures.h"
#include "matching2D.hpp"

namespace {

    void printNeighbourhoodStats(std::vector<cv::KeyPoint> &keypoints) {
        auto add_size = [](float sum, const cv::KeyPoint &kp) {
            return sum + kp.size;
        };
        const auto mean = std::accumulate(keypoints.begin(), keypoints.end(), 0.0F, add_size) / keypoints.size();

        auto add_square = [mean](float sum, const cv::KeyPoint &kp) {
            const auto d = kp.size - mean;
            return sum + d * d;
        };
        const auto total = std::accumulate(keypoints.begin(), keypoints.end(), 0.0F, add_square);
        const auto standardDeviation = total / keypoints.size();

        std::cout << "Neighbourhood size is " << mean << " +/- " << standardDeviation
                  << " for " << keypoints.size() << " keypoints." << std::endl;
    }
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    std::string dataPath = "../";

    // camera
    std::string imgBasePath = dataPath + "images/";
    std::string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    std::string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    constexpr int dataBufferSize = 2;   // no. of images which are held in memory (ring buffer) at the same time
    std::deque<DataFrame> dataBuffer{}; // list of data frames which are held in memory at the same time
    bool bVis = false;                  // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++) {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        std::ostringstream imgNumber;
        imgNumber << std::setfill('0') << std::setw(imgFillWidth) << imgStartIndex + imgIndex;
        std::string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if (dataBuffer.size() == dataBufferSize) {
            dataBuffer.pop_front();
        }
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        std::cout << "#1 : LOAD IMAGE INTO BUFFER done" << std::endl << std::endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        std::string detectorType = "SHITOMASI";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType == "SHITOMASI") {
            detectKeypointsShiTomasi(keypoints, imgGray, false);
        } else if (detectorType == "HARRIS") {
            detectKeypointsHarris(keypoints, imgGray, bVis);
        } else {
            detectKeypointsModern(keypoints, imgGray, detectorType, bVis);
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle.
        // NOTE: It absolutely makes sense to limit the search window first, then look for keypoints.
        //       However, it was a requirement of the project code to do it in this order.
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle) {
            const auto numKeypointsBefore = keypoints.size();
            const auto position = std::remove_if(keypoints.begin(), keypoints.end(),
                                                 [&vehicleRect](const cv::KeyPoint &kp) {
                                                     return !vehicleRect.contains(kp.pt);
                                                 });
            keypoints.erase(position, keypoints.end());

            const auto numKeypointsAfter = keypoints.size();
            std::cout << "Keeping " << numKeypointsAfter << " / " << numKeypointsBefore << " keypoints" << std::endl;
        }

        printNeighbourhoodStats(keypoints);

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        const auto bLimitKpts = false;
        if (bLimitKpts) {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") ==
                0) { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            std::cout << " NOTE: Keypoints have been limited!" << std::endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
        std::cout << "#2 : DETECT KEYPOINTS done" << std::endl << std::endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        std::string descriptorType = "BRISK"; // BRIEF, ORB, FREAK, AKAZE, SIFT

        describeKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors,
                          descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        std::cout << "#3 : EXTRACT DESCRIPTORS done" << std::endl << std::endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            std::vector<cv::DMatch> matches;
            std::string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
            std::string descriptorType = "DES_BINARY"; // DES_BINARY, DES_HOG
            std::string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            std::cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << std::endl << std::endl;

            // visualize matches between current and previous image
            bVis = true;
            if (bVis) {
                const auto drawFlags =
                        cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS |
                        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS;

                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                std::vector<char>(), drawFlags);

                std::string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                std::cout << "Press key to continue to next image" << std::endl << std::endl << std::endl;
                // cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

    return 0;
}

#include <iostream>
#include <iomanip>
#include <deque>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <numeric>
#include <fstream>

#include "dataStructures.h"
#include "matching2D.hpp"

namespace {

    void getNeighbourhoodStats(std::vector<cv::KeyPoint> &keypoints, float &mean, float &standardDeviation) {
        auto add_size = [](float sum, const cv::KeyPoint &kp) {
            return sum + kp.size;
        };
        mean = std::accumulate(keypoints.begin(), keypoints.end(), 0.0F, add_size) / keypoints.size();

        auto add_square = [mean](float sum, const cv::KeyPoint &kp) {
            const auto d = kp.size - mean;
            return sum + d * d;
        };
        const auto total = std::accumulate(keypoints.begin(), keypoints.end(), 0.0F, add_square);
        standardDeviation = total / keypoints.size();
    }
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[]) {

    constexpr auto bVisFinal = false;
    constexpr auto bVis = false;
    constexpr bool verbose = true;

    std::ofstream csv;
    csv.open("results.csv", std::ios::trunc);
    csv << "detector" << ", " << "descriptor" << ", " << "frame" << ", "
        << "detection duration" << ", " << "original keypoints" << ", "
        << "car keypoints" << ", " // after car segmentation
        << "neighbourhood mean" << ", " << "neighbourhood sigma" << ", " // neighbourhood
        << "description duration" << ", "
        << "matches" << ", "
        << "total duration" << "matches" << std::endl;

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

    /* MAIN LOOP OVER ALL IMAGES */
    const auto detectorTypes = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    const auto descriptorTypes = {"BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};

    for (const std::string &detectorType : detectorTypes) {
        for (const std::string &descriptorType : descriptorTypes) {

            // Nobody tells AKAZE what to do.
            if (descriptorType == "AKAZE" && detectorType != "AKAZE") {
                continue;
            }

            // Likewise, ORB and SIFT don't go along well.
            if (descriptorType == "ORB" && detectorType == "SIFT") {
                continue;
            }

            std::deque<DataFrame> dataBuffer{}; // list of data frames which are held in memory at the same time
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

                /* DETECT IMAGE KEYPOINTS */

                // extract 2D keypoints from current image
                std::vector<cv::KeyPoint> keypoints; // create empty feature list for current image

                //// STUDENT ASSIGNMENT
                //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
                //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
                // std::string detectorType = "BRISK";

                auto t = (double) cv::getTickCount();

                if (detectorType == "SHITOMASI") {
                    detectKeypointsShiTomasi(keypoints, imgGray, false, verbose);
                } else if (detectorType == "HARRIS") {
                    detectKeypointsHarris(keypoints, imgGray, bVis, verbose);
                } else {
                    detectKeypointsModern(keypoints, imgGray, detectorType, bVis, verbose);
                }

                const auto numKeypointsBefore = keypoints.size();
                const auto detectionDuration = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
                if (verbose) {
                    std::cout << detectorType + " detector with n= " << keypoints.size() << " keypoints in "
                              << 1000 * detectionDuration / 1.0
                              << " ms"
                              << std::endl;
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
                    const auto position = std::remove_if(keypoints.begin(), keypoints.end(),
                                                         [&vehicleRect](const cv::KeyPoint &kp) {
                                                             return !vehicleRect.contains(kp.pt);
                                                         });
                    keypoints.erase(position, keypoints.end());

                    const auto numKeypointsAfter = keypoints.size();
                    if (verbose) {
                        std::cout << "Keeping " << numKeypointsAfter << " / " << numKeypointsBefore << " keypoints"
                                  << std::endl;
                    }
                }

                float mean, standardDeviation;
                getNeighbourhoodStats(keypoints, mean, standardDeviation);
                if (verbose) {
                    std::cout << "Neighbourhood size is " << mean << " +/- " << standardDeviation
                              << " for " << keypoints.size() << " keypoints." << std::endl;
                }



                //// EOF STUDENT ASSIGNMENT

                // optional : limit number of keypoints (helpful for debugging and learning)
                const auto bLimitKpts = false;
                if (bLimitKpts) {
                    int maxKeypoints = 50;

                    if (detectorType ==
                        "SHITOMASI") { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                        keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
                    }
                    cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
                    if (verbose) {
                        std::cout << " NOTE: Keypoints have been limited!" << std::endl;
                    }
                }

                // push keypoints and descriptor for current frame to end of data buffer
                (dataBuffer.end() - 1)->keypoints = keypoints;

                /* EXTRACT KEYPOINT DESCRIPTORS */

                //// STUDENT ASSIGNMENT
                //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
                //// -> BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
                // std::string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

                cv::Mat descriptors;

                t = (double) cv::getTickCount();

                describeKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors,
                                  descriptorType, verbose);

                const auto descriptionDuration = ((double) cv::getTickCount() - t) / cv::getTickFrequency();
                if (verbose) {
                    std::cout << descriptorType << " descriptor extraction in " << 1000 * descriptionDuration / 1.0
                              << " ms"
                              << std::endl;
                }

                //// EOF STUDENT ASSIGNMENT

                // push descriptors for current frame to end of data buffer
                (dataBuffer.end() - 1)->descriptors = descriptors;

                std::vector<cv::DMatch> matches;
                if (dataBuffer.size() > 1) // wait until at least two images have been processed
                {

                    /* MATCH KEYPOINT DESCRIPTORS */

                    std::string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
                    std::string descriptorVariant = (descriptorType == "SIFT") ? "DES_HOG"
                                                                               : "DES_BINARY"; // DES_BINARY, DES_HOG
                    std::string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

                    //// STUDENT ASSIGNMENT
                    //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
                    //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

                    matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                     (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                     matches, descriptorVariant, matcherType, selectorType, verbose);

                    if (verbose) {
                        std::cout << "Number of matches: " << matches.size() << std::endl;
                    }

                    //// EOF STUDENT ASSIGNMENT

                    // store matches in current data frame
                    (dataBuffer.end() - 1)->kptMatches = matches;

                    // visualize matches between current and previous image
                    if (bVisFinal) {
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

                        if (verbose) {
                            std::cout << "Press key to continue to next image" << std::endl;
                        }
                        cv::waitKey(0); // wait for key to be pressed
                    }
                }

                if (verbose) {
                    std::cout << std::endl << std::endl;
                }

                // Final CSV line
                csv << detectorType << ", " << descriptorType << ", " << imgIndex << ", "
                    << detectionDuration << ", " << numKeypointsBefore << ", "
                    << keypoints.size() << ", " // after car segmentation
                    << mean << ", " << standardDeviation << ", " // neighbourhood
                    << descriptionDuration << ", "
                    << matches.size() << ", "
                    << (detectionDuration + descriptionDuration) << std::endl;

            } // eof loop over all images

            csv.flush();
        }
    }

    csv.close();
    return 0;
}

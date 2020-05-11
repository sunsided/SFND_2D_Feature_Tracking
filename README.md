# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project.
As a preparation for this, you will now build the feature tracking part and test various detector / descriptor
combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to
  optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard
  to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN
  approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations
  and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with
this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on
integrating LiDAR points and on object detection using deep-learning. 

## Build instructions

### The basic steps

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

### Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing 
    the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Feature processing summary

The following feature detectors were implemented:

- Shi-Thomasi ("Good Features To Track")
- Harris corners
- FAST
- BRISK
- ORB
- AKAZE
- SIFT

### Number of features found on tracked car

The following gives an overview of the number of keypoints found per individual video frame;
note that only the region of the car driving in front of the ego car is considered.
Note also that these values are obtained using the detector's respective default configurations
and no hyper-parameter tuning was performed.
For the Harris detector, a minimum response of 30% was required and zero overlap was permitted
during non-maximum suppression. 

| Detector    |  #0 |  #1 |  #2 |  #3 |  #4 |  #5 |  #6 |  #7 |  #8 |  #9 | Average      | Rank  |
|-------------|-----|-----|-----|-----|-----|-----|-----|-----|-----|-----|--------------|-------|
| Shi-Thomasi | 125 | 118 | 123 | 120 | 120 | 113 | 114 | 123 | 111 | 112 | 117.9 ± 4.8  | 5     |
| Harris      |  22 |  23 |  27 |  27 |  33 |  30 |  28 |  37 |  31 |  26 |  28.4 ± 4.3  | 7     |
| FAST        | 149 | 152 | 150 | 155 | 149 | 149 | 156 | 150 | 138 | 143 | 149.1 ± 5.0  | **3** |
| BRISK       | 264 | 282 | 282 | 277 | 297 | 279 | 289 | 272 | 266 | 254 | 276.2 ± 12.0 | **1** |
| ORB         |  92 | 102 | 106 | 113 | 109 | 125 | 130 | 129 | 127 | 128 | 116.1 ± 12.8 | 6     |
| AKAZE       | 166 | 157 | 161 | 155 | 163 | 164 | 173 | 175 | 177 | 179 | 167.0 ± 8.1  | **2** |
| SIFT        | 138 | 132 | 124 | 137 | 134 | 140 | 137 | 148 | 159 | 137 | 138.6 ± 8.9  | 4     |

### Neighbourhood sizes

The neighbourhood size describes the diameter across each keypoint location that was considered
during detection. For the Harris detector, the configured block size was 4, which results in a
three pixel radius around the center point; as a result, the reported neighbourhood size is six. 

| Frame       |  Shi-Thomasi |  Harris |  FAST |  BRISK |  ORB |  AKAZE |  SIFT |
|-------------|--------------|---------|-------|--------|------|--------|-------|
| #0          | 4 | 6 | 7 | 21.5 ± 212.4 | 57.0 ± 661.0 | 7.7 ± 15.4 | 4.9 ± 35.1 |
| #1          | 4 | 6 | 7 | 21.7 ± 212.1 | 57.2 ± 680.5 | 7.4 ± 12.4 | 5.1 ± 38.1 |
| #2          | 4 | 6 | 7 | 21.6 ± 191.0 | 56.4 ± 672.1 | 7.4 ± 12.6 | 4.9 ± 36.2 |
| #3          | 4 | 6 | 7 | 20.3 ± 159.2 | 55.1 ± 629.7 | 7.5 ± 11.9 | 4.7 ± 27.4 | 
| #3          | 4 | 6 | 7 | 22.5 ± 220.7 | 56.7 ± 625.6 | 7.7 ± 11.8 | 4.7 ± 30.3 |
| #5          | 4 | 6 | 7 | 22.9 ± 249.9 | 56.6 ± 596.7 | 7.6 ± 11.4 | 4.7 ± 31.0 |
| #6          | 4 | 6 | 7 | 21.8 ± 215.3 | 56.7 ± 646.3 | 7.7 ± 11.8 | 5.4 ± 42.4 |
| #7          | 4 | 6 | 7 | 22.1 ± 226.0 | 55.4 ± 611.8 | 7.8 ± 12.3 | 4.6 ± 26.4 |
| #8          | 4 | 6 | 7 | 22.5 ± 230.7 | 54.6 ± 638.0 | 7.8 ± 12.2 | 5.5 ± 44.4 |
| #9          | 4 | 6 | 7 | 22.0 ± 215.1 | 54.3 ± 560.2 | 7.8 ± 12.9 | 5.6 ± 44.6 |
| | | | | | | | |
| Average     | 4 | 6 | 7 | 21.9 ± 214.5 | 56.0 ± 633.1 | 7.6 ± 12.5 | 5.0 ± 36.2 |
| Rank        | 6 | 5 | 4 | **3**        | 7            | **1**      | **2**      |

From these values we can see that Shi-Thomasi, Harris and FAST keypoints utilize a fixed
neighbourhood size, whereas BRISK, ORB, AKAZE and SIFT make use of different scales.
It is not obvious how to rank these results, although there are a few intuitions to apply:

- Fixes-size keypoints are faster to compute, but their rigid structure makes them likely
  to be more useful only for lateral or vertical movement (parallel to the image plane).
- Keypoints of dynamic scale are helpful to track objects that move either towards or
  away from the camera, as the keypoint may adapt to the perceived scale of the object.
- Small keypoints should be able to capture details in a more meaningful way, as
  opposed to relatively global image features.
- Likewise, too big features (as can be seen in the case of ORB) may be good for re-identification
  for keypoints across drastic image changes, but are likely to sacrifice position accuracy.

From this, it seems reasonable to settle for either AKAZE, SIFT or BRISK.

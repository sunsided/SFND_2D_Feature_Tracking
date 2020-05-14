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

| Frame    | Shi-Thomasi | Harris     | FAST        | BRISK        | ORB          | AKAZE       | SIFT        |
|----------|-------------|------------|-------------|--------------|--------------|-------------|-------------|
| #0       | 125         | 22         | 149         | 264          | 92           | 166         | 138         |
| #1       | 118         | 23         | 152         | 282          | 102          | 157         | 132         |
| #2       | 123         | 27         | 150         | 282          | 106          | 161         | 124         |
| #3       | 120         | 27         | 155         | 277          | 113          | 155         | 137         |
| #4       | 120         | 33         | 149         | 297          | 109          | 163         | 134         |
| #5       | 113         | 30         | 149         | 279          | 125          | 164         | 140         |
| #6       | 114         | 28         | 156         | 289          | 130          | 173         | 137         |
| #7       | 123         | 37         | 150         | 272          | 129          | 175         | 148         |
| #8       | 111         | 31         | 138         | 266          | 127          | 177         | 159         |
| #9       | 112         | 26         | 143         | 254          | 128          | 179         | 137         |
| | | | | | | | |
| Average  | 117.9 ± 4.8 | 28.4 ± 4.3 | 149.1 ± 5.0 | 276.2 ± 12.0 | 116.1 ± 12.8 | 167.0 ± 8.1 | 138.6 ± 8.9 |
| Rank     | 5           | 7          | **3**       | **1**        | 6            | **2**       | 4           |

The detectors are ranked in descending order of number of keypoints created. The intuition here is
that the more (meaningful) descriptors a detector is able to create, the more exact our understanding
of the world will be. It is important to mention however that these numbers are the number of keypoints
prior to matching.

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

## Number of matches

| detector  |  descriptor | 0→1 | 1→2 | 2→3 | 3→4 | 4→5 | 5→6 | 6→7 | 7→8 | 8→9 | Average      |
|-----------|-------------|-----|-----|-----|-----|-----|-----|-----|-----|-----|--------------|
| AKAZE     |  AKAZE      | 138 | 138 | 133 | 127 | 129 | 146 | 147 | 151 | 150 | 125.9 ± 42.7 |
|           |  BRIEF      | 141 | 134 | 131 | 130 | 134 | 146 | 150 | 148 | 152 | 126.6 ± 42.9 |
|           |  BRISK      | 137 | 125 | 129 | 129 | 131 | 132 | 142 | 146 | 144 | 121.5 ± 41.1 |
|           |  FREAK      | 126 | 129 | 127 | 121 | 122 | 133 | 144 | 147 | 138 | 118.7 ± 40.4 |
|           |  ORB        | 130 | 129 | 128 | 115 | 132 | 132 | 137 | 137 | 146 | 118.6 ± 40.2 |
|           |  SIFT       | 148 | 143 | 139 | 140 | 143 | 151 | 157 | 159 | 159 | 133.9 ± 45.2 |
|           |             |     |     |     |     |     |     |     |     |     |              |
| BRISK     |  BRIEF      | 178 | 205 | 185 | 179 | 183 | 195 | 207 | 189 | 183 | 170.4 ± 57.6 |
|           |  BRISK      | 171 | 176 | 157 | 176 | 174 | 188 | 173 | 171 | 184 | 157 ± 52.9   |
|           |  FREAK      | 160 | 177 | 155 | 173 | 161 | 183 | 169 | 178 | 168 | 152.4 ± 51.5 |
|           |  ORB        | 160 | 171 | 157 | 170 | 154 | 180 | 171 | 175 | 172 | 151 ± 50.9   |
|           |  SIFT       | 218 | 231 | 225 | 218 | 223 | 228 | 230 | 205 | 208 | 198.6 ± 66.7 |
|           |             |     |     |     |     |     |     |     |     |     |              |
| FAST      |  BRIEF      | 119 | 130 | 118 | 126 | 108 | 123 | 131 | 125 | 119 | 109.9 ± 37.2 |
|           |  BRISK      | 97  | 104 | 101 | 98  | 85  | 107 | 107 | 100 | 100 | 89.9 ± 30.5  |
|           |  FREAK      | 98  | 99  | 91  | 98  | 85  | 99  | 102 | 101 | 105 | 87.8 ± 29.8  |
|           |  ORB        | 122 | 122 | 115 | 129 | 107 | 120 | 126 | 122 | 118 | 108.1 ± 36.5 |
|           |  SIFT       | 132 | 131 | 126 | 138 | 135 | 136 | 135 | 133 | 120 | 118.6 ± 39.8 |
|           |             |     |     |     |     |     |     |     |     |     |              |
| HARRIS    |  BRIEF      | 20  | 22  | 26  | 25  | 29  | 24  | 25  | 32  | 24  | 22.7 ± 8.2   |
|           |  BRISK      | 18  | 17  | 21  | 22  | 24  | 20  | 23  | 29  | 27  | 20.1 ± 7.6   |
|           |  FREAK      | 17  | 20  | 21  | 19  | 22  | 27  | 22  | 27  | 24  | 19.9 ± 7.3   |
|           |  ORB        | 19  | 20  | 25  | 25  | 29  | 25  | 26  | 33  | 26  | 22.8 ± 8.5   |
|           |  SIFT       | 20  | 22  | 27  | 24  | 31  | 28  | 25  | 34  | 29  | 24 ± 8.9     |
|           |             |     |     |     |     |     |     |     |     |     |              |
| ORB       |  BRIEF      | 49  | 43  | 45  | 59  | 53  | 78  | 68  | 84  | 66  | 54.5 ± 22.3  |
|           |  BRISK      | 73  | 74  | 79  | 85  | 79  | 92  | 90  | 88  | 91  | 75.1 ± 25.9  |
|           |  FREAK      | 42  | 36  | 44  | 47  | 44  | 51  | 52  | 48  | 56  | 42 ± 15.0    |
|           |  ORB        | 65  | 69  | 71  | 85  | 91  | 101 | 95  | 93  | 91  | 76.1 ± 27.9  |
|           |  SIFT       | 76  | 87  | 88  | 91  | 89  | 104 | 105 | 101 | 104 | 84.5 ± 29.6  |
|           |             |     |     |     |     |     |     |     |     |     |              |
| SHITOMASI |  BRIEF      | 115 | 111 | 104 | 101 | 102 | 102 | 100 | 109 | 100 | 94.4 ± 31.8  |
|           |  BRISK      | 95  | 88  | 80  | 90  | 82  | 79  | 85  | 86  | 82  | 76.7 ± 26.0  |
|           |  FREAK      | 86  | 90  | 86  | 88  | 86  | 80  | 81  | 86  | 85  | 76.8 ± 25.8  |
|           |  ORB        | 104 | 103 | 100 | 102 | 103 | 98  | 98  | 102 | 97  | 90.7 ± 30.3  |
|           |  SIFT       | 115 | 113 | 107 | 109 | 104 | 105 | 103 | 113 | 103 | 97.2 ± 32.7  |
|           |             |     |     |     |     |     |     |     |     |     |              |
| SIFT      |  BRIEF      | 86  | 78  | 76  | 85  | 69  | 74  | 76  | 70  | 88  | 70.2 ± 24.2  |
|           |  BRISK      | 64  | 66  | 62  | 66  | 59  | 64  | 64  | 67  | 80  | 59.2 ± 20.4  |
|           |  FREAK      | 65  | 72  | 64  | 66  | 59  | 59  | 64  | 65  | 79  | 59.3 ± 20.5  |
|           |  SIFT       | 94  | 95  | 89  | 107 | 99  | 93  | 94  | 115 | 114 | 90 ± 31.2    |

## Execution times

The following table gives the execution times of keypoint detection and description
as the median time in seconds measured across all video frames.

| detector  |  descriptor | Detector [s] | Descriptor [s] | Total [s] |
|-----------|-------------|--------------|----------------|-----------|
| AKAZE     |  AKAZE      | 0.0370       | 0.0321         | 0.0702    |
|           |  BRIEF      | 0.0409       | 0.0008         | 0.0420    |
|           |  BRISK      | 0.0400       | 0.1553         | 0.1954    |
|           |  FREAK      | 0.0368       | 0.0218         | 0.0581    |
|           |  ORB        | 0.0406       | 0.0058         | 0.0463    |
|           |  SIFT       | 0.0384       | 0.0139         | 0.0538    |
|           |             |              |                |           |
| BRISK     |  BRIEF      | 0.1860       | 0.0006         | 0.1866    |
|           |  BRISK      | 0.1849       | 0.1566         | 0.3416    |
|           |  FREAK      | 0.1858       | 0.0214         | 0.2072    |
|           |  ORB        | 0.1866       | 0.0081         | 0.1948    |
|           |  SIFT       | 0.1847       | 0.0179         | 0.2030    |
|           |             |              |                |           |
| FAST      |  BRIEF      | 0.0009       | 0.0004         | 0.0013    |
|           |  BRISK      | 0.0009       | 0.1558         | 0.1567    |
|           |  FREAK      | 0.0009       | 0.0204         | 0.0213    |
|           |  ORB        | 0.0009       | 0.0030         | 0.0038    |
|           |  SIFT       | 0.0009       | 0.0104         | 0.0113    |
|           |             |              |                |           |
| HARRIS    |  BRIEF      | 0.0105       | 0.0002         | 0.0108    |
|           |  BRISK      | 0.0071       | 0.1538         | 0.1609    |
|           |  FREAK      | 0.0063       | 0.0199         | 0.0262    |
|           |  ORB        | 0.0065       | 0.0022         | 0.0087    |
|           |  SIFT       | 0.0065       | 0.0085         | 0.0149    |
|           |             |              |                |           |
| ORB       |  BRIEF      | 0.0054       | 0.0003         | 0.0058    |
|           |  BRISK      | 0.0054       | 0.1555         | 0.1610    |
|           |  FREAK      | 0.0055       | 0.0202         | 0.0259    |
|           |  ORB        | 0.0054       | 0.0083         | 0.0137    |
|           |  SIFT       | 0.0055       | 0.0201         | 0.0255    |
|           |             |              |                |           |
| SHITOMASI |  BRIEF      | 0.0140       | 0.0011         | 0.0151    |
|           |  BRISK      | 0.0122       | 0.1836         | 0.1958    |
|           |  FREAK      | 0.0082       | 0.0202         | 0.0284    |
|           |  ORB        | 0.0124       | 0.0034         | 0.0156    |
|           |  SIFT       | 0.0081       | 0.0092         | 0.0173    |
|           |             |              |                |           |
| SIFT      |  BRIEF      | 0.0732       | 0.0007         | 0.0739    |
|           |  BRISK      | 0.0723       | 0.1483         | 0.2212    |
|           |  FREAK      | 0.0734       | 0.0207         | 0.0944    |
|           |  SIFT       | 0.0554       | 0.0455         | 0.1013    |

From each block, we will now only keep the two highest-performing (lowest duration)
option. This gives:

| detector  |  descriptor | Detector [s] | Descriptor [s] | Total [s] |
|-----------|-------------|--------------|----------------|-----------|
| AKAZE     |  BRIEF      | 0.0409       | 0.0008         | 0.0420    |
|           |  ORB        | 0.0406       | 0.0058         | 0.0463    |
|           |             |              |                |           |
| BRISK     |  BRIEF      | 0.1860       | 0.0006         | 0.1866    |
|           |  ORB        | 0.1866       | 0.0081         | 0.1948    |
|           |             |              |                |           |
| FAST      |  BRIEF      | 0.0009       | 0.0004         | 0.0013    |
|           |  ORB        | 0.0009       | 0.0030         | 0.0038    |
|           |             |              |                |           |
| HARRIS    |  BRIEF      | 0.0105       | 0.0002         | 0.0108    |
|           |  ORB        | 0.0065       | 0.0022         | 0.0087    |
|           |             |              |                |           |
| ORB       |  BRIEF      | 0.0054       | 0.0003         | 0.0058    |
|           |  ORB        | 0.0054       | 0.0083         | 0.0137    |
|           |             |              |                |           |
| SHITOMASI |  BRIEF      | 0.0140       | 0.0011         | 0.0151    |
|           |  ORB        | 0.0124       | 0.0034         | 0.0156    |
|           |             |              |                |           |
| SIFT      |  BRIEF      | 0.0732       | 0.0007         | 0.0739    |
|           |  FREAK      | 0.0734       | 0.0207         | 0.0944    |

It is pretty clear that BRIEF and ORB are the two options to look at when it comes
to speed; in the case of SIFT, where ORB doesn't work, the second-best option is FREAK.
It is noteworthy that for the Harris detector, ORB is the faster option, whereas
in any other hase, BRIEF outperforms ORB slightly in total. It is also apparent
that the total duration is massively governed by the detection step whereas
the keypoint description generally doesn't matter much.

## Detector / Descriptor combination

From the above numbers we find that

- FAST keypoints with BRIEF descriptors outperform every other combination by
  a large margin. This combination generated around 100 keypoint matches
  on average on the tracked car alone. However, FAST uses a fixed-size window
  which may make it problematic when scale changes a lot. If we sample pictures
  at a high enough rate, this may not be much of an issue.
- FAST keypoints with ORB descriptors are next in line with mainly the same arguments.
  However, ORB keypoints tend to get relatively large in size, which may be inappropriate
  for tracking closeup objects.

Then,

- ORB keypoints with a BRIEF detector are reasonably fast, but only generate about
  half the number of keypoints per frame on the tracked car.  
- Interestingly, the Harris corner detector with a naively implemented (somewhat O(n²)-y)
  non-maximum suppression algorithm performs in an outstanding way - however,
  the Harris detector only yielded about 30 keypoints per frame, which may ultimately be
  too few if the conditions get rough (light changes, perspective shifts, etc.).

Given that we are building a collision avoidance system, time is crucial. At high speeds
or in drastic deceleration scenarios, being able to process the environment faster makes
all the difference. Given that even a constant acceleration model of movement is an
oversimplification, being able to run ten times the number of measurements will help
to make the right decisions.

As such, the recommendations are the following:

- FAST with BRIEF descriptors,
- FAST with ORB descriptors (or both), as well as
- ORB with BRIEF descriptors

are candidates worth looking at. ORB/ORB, although it literally consists of FAST and BRIEF, didn't make it.

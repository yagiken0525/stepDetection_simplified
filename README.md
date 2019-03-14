Step Detection using Single RGB Camera
====

Measuring how people walk using a single RGB camera.

![result](https://github.com/yagiken0525/stepDetection_simplified/blob/media/demo.gif)

## Description
This method can measure step positions in 3D scale. Stride length, step width, step timing, walking speed and walking distance are measurable. A camera is supposed to be fixed at the position where whole body of the target people are captured. Step position and timing are measured using OpenPose.

## Demo
set parameters in "FootPrint.h" as following
- USE_CHECKER_BOARD: true
- PIXEL_SCALE: 0.1
- VOTE_RANGE: 5
- STEP_THRESHOLD: 10
then run.

## Requirement
openpose
https://github.com/CMU-Perceptual-Computing-Lab/openpose

## Usage 
Preparation
```console
$ mkdir ./projects/NEW_PROJECTNAME
$ mkdir ./projects/NEW_PROJECTNAME/videos
$ cp INPUT_VIDEO ./projects/NEW_PROJECTNAME/videos/
```

Homograpy Estimation
1. Estimate Homography matrix usign calibration board
   - set USE_CHECKER_BOARD: true
   - prepare calibration.mp4
2. Estimate Homography matrix by clicking
   - set USE_CHECKER_BOARD: false
   - prepare cornerPoints.txt 
   
Background Image
   - prepare "background.mp4" or "backGround.jpg"
   

## Install
1. build openpose following https://github.com/CMU-Perceptual-Computing-Lab/openpose.
2. copy "openpose/models" to the directory that executable file exists.

## Author
Kntaro Yagi (yagiken525@keio.jp)

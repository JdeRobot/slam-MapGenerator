# slam-MapGenerator
## Project Description
- This project aims at creating realistic 3D map from online SLAM algorithm.
- This documentation is more developer amd user oriented.
- You may refer to the [project wiki](https://jderobot.org/Club-jianxiong) for more progress-oriented content. (More Images and Videos there)
<!-- more description with images here -->

## Overview 
This project splits into 3 major parts:
- Loop Closing
- Global Bandle Ajustment (BA)
- Reconstructing 3D Map 

Each part coresponding to separate execuable files for now. I will put them into one execute files with nice config in the final phase of GSOC.

## Current Progress
The loop closing has been implemented and is working fine now. 
Reconstructing 3D Map is under developement. I have got a very naive prototype and is working on improving it.

## Dataset and Result
All dataset and result will be uploaded to google drive soon. (in one or two days)

### TUM Dataset
If you are using the monocular dataset from TUM, which is fisheye camera, you need to first get undistorted images. You may do this by using the code in the dataset webpage.

## Detail Usage
This section talks about the detail usage of the current working code (under master branch).

### Dependency
slam-MapGen requires the following dependency:
- cmake
- Eigen3
- OpenCV (3.0 or upper)
- PCL
- [DBoW2](https://github.com/dorian3d/DBoW2)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [Ceres Solver](https://github.com/ceres-solver/ceres-solver)

To install cmake, eigen3:
```shell
sudo apt-get install cmake libeigen3-dev
```
For other dependency, you may follow the official installation guide. <br/>
Note: It is recommended to compile OpenCV3 from source and install. Using the anaconda-version with OpenCV3 will raise errors.

### Compile
```shell
mkdir build

cd build

# If you don't want to view the debug info
cmake -DCMAKE_BUILD_TYPE=Release ..

make -j4
```
### Loop Closing
```shell
./loop_detection ../config/loop_detector_config.yaml
```
This would output 2 files in the working directory: poses_original.yaml and poses_optimized.yaml. <br/>
You may refer to visualize it by using [slam-viewer](https://github.com/JdeRobot/slam-viewer).
```shell
YOUR_PATH_TO_SLAM_VIEWER/Viewer ~/SLAM/slam-MapGenerator/build/poses_optimized.yaml
```

##### Config File: 
The second parameters of the above command is the config file, it's in the following format:

| Parameter Name           | Explaination                                                                               |
|--------------------------|--------------------------------------------------------------------------------------------|
| img_dir                  | The image directory (undistorted images)                                                   |
| trajectory               | The input trajectory file                                                                  |
| Vocabulary               | The pre-trained Visual Vocabulary File (You may get this through the above dataset link)   |
| loop_detection_threshold | The threshold for deciding whether two frames are similar enough to be a loop closing pair |



### Surface Reconstruction
TODO: write usage and documentation about it.

### Testing environment
All programs are tested under Ubuntu16.04. 

<!-- Add wiki link -->
<!--
## Acknowledgment
This project is also a student program for GSOC 2018 (Google Summer of Code) from May 14th, 2018 to XXX Date. Detailed wiki for GSOC 2018 on this project is here. 
-->

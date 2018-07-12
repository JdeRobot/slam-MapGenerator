# slam-MapGenerator
## Project Description
- This project aims at creating realistic 3D map from online SLAM algorithm.
- This documentation is more developer and user oriented.
- You may refer to the [project wiki](https://jderobot.org/Club-jianxiong) for more progress-oriented content. (More images and videos there)
<!-- more description with images here -->

## Overview 
This project splits into 3 major parts:
- Loop Closing
- Global Bundle Adjustment (BA)
- Reconstructing 3D Map 

Each part corresponding to separate executable files for now. I will put them into one execute files with nice configure in the final phase of GSOC.

## Current Progress
The loop closing has been implemented and is working fine now. 
Reconstructing 3D Map is under development. I have got a very naive prototype and is working on improving it.

## Dataset and Result
The detailed README about datasets is in the subfolder datasets. <br/>
All datasets and some results are available at [Google Drive](https://drive.google.com/drive/folders/1Ta6QgiQ5ASHj42pyGt9OijQ7t_oUynFb). If the link doesn't work, you may type it manually: https://drive.google.com/drive/folders/1Ta6QgiQ5ASHj42pyGt9OijQ7t_oUynFb

### TUM Dataset
If you are using the monocular dataset from TUM, which is fish-eye camera, you need to first get undistorted images. You may do this by using the code in the dataset webpage.

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

##### Configure File: 
The second parameters of the above command is the configure file, it's in the following format:

| Parameter Name           | Explanation                                                                               |
|--------------------------|--------------------------------------------------------------------------------------------|
| img_dir                  | The image directory (undistorted images)                                                   |
| trajectory               | The input trajectory file                                                                  |
| Vocabulary               | The pre-trained Visual Vocabulary File (You may get this through the above dataset link)   |
| loop_detection_threshold | The threshold for deciding whether two frames are similar enough to be a loop closing pair |



### Surface Reconstruction
```shell
./surface_recon ../config/surface_recon_config.yaml
```
This will save the reconstructed surface to mesh.vtk in the current working directory. 

##### Configure File: 
The second parameters of the above command is the configure file, it's in the following format:

| Parameter Name               | Explanation                                                                               |
|------------------------------|--------------------------------------------------------------------------------------------|
| use_trajectory               | Set 1 to use the trajectory file, otherwise set 0 to use PCD pointcloud                    |
| trajectory                   | The input trajectory file                                                                  |
| pointcloud                   | The input pointcloud file                                                                  |
| use_fast_triangulation_recon | Set 1 to use the method of fast triangulation (the native but useful one)                  |
| use_poisson_recon            | Set 1 to use the method of Poisson R
econstruction                                          |

### Testing environment
All programs are tested under Ubuntu16.04. 

### Contact
We are using Github Issues to manage bugs and provide answers to questions. Feel free to file bug report / ask questions.

<!-- Add wiki link -->
<!--
## Acknowledgment
This project is also a student program for GSOC 2018 (Google Summer of Code) from May 14th, 2018 to XXX Date. Detailed wiki for GSOC 2018 on this project is here. 
-->

# slam-MapGenerator
## Project Description
Slam-MapGenerator is a complete system to reconstruct surface from visual odometry (provided by online sparse SLAM / VO algorithm) + raw images.
<!-- move the following link to the bottom after GSOC, we are going to do something more in the future, the wiki could be miss leading at that time -->
</br>
For GSOC, You may refer to the [project wiki](https://jderobot.org/Club-jianxiong) for more progress-oriented content. (More images and videos there)
<!-- more description with images here -->

## Overview 
This project contains 3 major parts for now:
- Loop Closing
- Global Bundle Adjustment (BA)
- Reconstructing 3D Map 


### Current Progress
All 3 parts has finished development and testing. The whole program is capable of handing surface reconstruction in simple environment.
</br>

More complex environment would require further development.
We would discuss and continue further development after August 22rd, 2018. Feature enhancement and suggestions are more than welcomed.
</br>


## Dataset and Result
The detailed README about datasets is in the subfolder datasets. <br/>
All datasets and some results are available at [Google Drive](https://drive.google.com/drive/folders/1Ta6QgiQ5ASHj42pyGt9OijQ7t_oUynFb). If the link doesn't work, you may type it manually: https://drive.google.com/drive/folders/1Ta6QgiQ5ASHj42pyGt9OijQ7t_oUynFb

### TUM Dataset
If you are using the monocular dataset from TUM, which is fish-eye camera, you need to get undistorted images. You may do this by using the code provided in the TUM dataset webpage.

## Detail Usage
### Dependency
slam-MapGen requires the following dependency:
- cmake
- Eigen3
- OpenCV (3.0 or upper)
- [PCL Library]
(http://www.pointclouds.org)
- [DBoW2](https://github.com/dorian3d/DBoW2)
- [DLib](https://github.com/dorian3d/DLib)
- [Pangolin](https://github.com/stevenlovegrove/Pangolin)
- [Ceres Solver](https://github.com/ceres-solver/ceres-solver)


To install cmake, eigen3:
```shell
sudo apt-get install cmake libeigen3-dev
```
For other dependency, you may follow the official installation guide. <br/>
Note: It is recommended to compile OpenCV3 from source and install. Using the anaconda-version with OpenCV3 will raise errors sometimes.

### Compile
```shell
mkdir build

cd build

# If you don't want to view the debug info
cmake -DCMAKE_BUILD_TYPE=Release ..

make -j4
```

### Run
#### download the dataset
- Get the dataset from my google drive, choose TUM-RGBD-freiburg3-far, [short-cut](https://drive.google.com/file/d/1aOcYjd7-RJUvxnybc-ldJJpaYPsGLtSv/view?usp=sharing)
- Download corresponding images from TUM (Sequence 'freiburg3_structure_texture_far'). [short-cut](https://vision.in.tum.de/data/datasets/rgbd-dataset/download#freiburg3_structure_texture_far)
#### run the code
```shell
./build/map_generator ./config/example.yaml
```
The final reconstructed surface will be visualized with GUI. <br/>
pose-graph optimization result will be saved to pose_optimized.yaml (if enabled). Global Bundle Adjustment result will be saved to BA_optimized.yaml (if enabled). You may visualized them by [slam-viewer](https://github.com/JdeRobot/slam-viewer.git) 

##### Configure File: 
The second parameters of the above command is the configure file, it's in the following format: <br/>
The configure file has 4 blocks, detailed explanation are following:

###### Common:
| Parameter Name           | Explanation                                                                                |
|--------------------------|--------------------------------------------------------------------------------------------|
| img_dir                  | The image directory (undistorted images)                                                   |
| trajectory               | The input trajectory file                                                                  |
| enableLoopClosure        | Set to 1 to enable loop closure, set to 0 otherwise                                        |
| enableBA        		   | Set to 1 to enable global bundle adjustment, set to 0 otherwise                            |
| enableSurfaceRecon       | Set to 1 to enable surface reconstruction, set to 0 otherwise                              |

Note: *${img_dir} + filename in trajectory file (YAML)* should be the fullpath of the image file (JPG/PNG/BMP). <br/>
This is the reason that img_dir is written as /home/freiburg3_far/rgb/../

###### LoopClosure:
| Parameter Name           | Explanation                                                                                |
|--------------------------|--------------------------------------------------------------------------------------------|
| Vocabulary               | The pre-trained Visual Vocabulary File (You may get this through the above dataset link)   |
| loop_detection_threshold | The threshold for deciding whether two frames are similar enough to be a loop closing pair |

###### SurfaceRecon:
| Parameter Name           | Explanation                                                                                                   |
|--------------------------|---------------------------------------------------------------------------------------------------------------|
| reconMethod              | Only PlaneRANSAC+GreedyTriangulation is a reliable method working on test set for now.                        |
| enableStitchingImage     | Set 1 to enable stitching image to texture on visualization, this would discard plane unable to be visualized. A plane is considered valid only if all 3 vertexes of the triangle appear on same single image |

###### PlaneRANSAC:
| Parameter Name           | Explanation                                                                                                   |
|--------------------------|---------------------------------------------------------------------------------------------------------------|
| minPreserveRatio         | The minimum inlier ratio RANSAC is going to preserve                                                          |

###### GreedyTriangulation:
| Parameter Name           | Explanation                                                                                                   |
|--------------------------|---------------------------------------------------------------------------------------------------------------|
| mu                       | the multiplier of the nearest neighbor distance to obtain the final search radius for each point              |
| maximumNearestNeighbors  | the maximum number of nearest neighbors to be searched for.                                                   |
| searchRadius             | the sphere radius used for determining the k-nearest neighbors                                                |
Note: detailed explanation refers to PCL Library  GreedyTriangulation method

### Testing environment
All programs are tested under Ubuntu16.04. 

<!-- ### Mutli-threading
Since this project depends on ceres-solver, if you want multi-threading, make sure that you built the ceres-solver with OpenMP / TBB multi-threading.
 -->
### Contact
We are using Github Issues to manage bugs and provide answers to questions. 
Feel free to file bug report / ask questions there.
New feature request and discussion are also more than welcomed.


## Acknowledgment
This project was proposed and accepted as a student program for GSOC 2018 (Google Summer of Code), from May 14th to August 14th, 2018. Detailed wiki for this project as GSOC 2018 on this project is on the [project wiki](https://jderobot.org/Club-jianxiong). 


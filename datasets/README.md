# Datasets
## Description
This folder contains all the pre-processed datasets used in this project. All the pre-processing steps are described in detail. <br/>
In addition, pre-trained visual vocabulary is stored to the folder voc. <br.>

## Images Datasets
### [Monocular Visual Odometry Dataset](https://vision.in.tum.de/data/datasets/mono-dataset) Sequence 43
- This dataset contains distorted images, so we use its supporting code to undistort images and saved to the folder undistorted_images. <br/>
- This dataset is used for loop closing (including loop detection and loop correction) and surface reconstruction via DSO + Fast Triangulation
- Only undistroted images are stored here, you may get other datasets from the official site

### RGBD-Datasets
- No preprocess required for this dataset.

## Visual Vocabulary
The visual vocabulary is provided in 2 formats, you may use either one:
- File format used in ORB2_SLAM
- tar.gz (Do not untar it, you can read it directly with DBoW2 library, with cv::FileStorage format)
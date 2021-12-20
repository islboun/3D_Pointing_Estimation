# Detecting 3D Hand Pointing Directionfrom RGB-D Data in Wide-Ranging HRI Scenarios



This repository contains the codes for our HRI 2022 paper titled as "Detecting 3D Hand Pointing Direction from RGB-D Data in Wide-Ranging HRI Scenarios".

Pointing Gesture & 3D Pointing Direction Data Sets can be downloaded from links below.

## Abstract
This paper addresses the detection of 3D hand point-ing  direction  from  RGB-D  data  by  a  mobile  robot.  Consideringubiquitous forms of pointing gestures, the 3D pointing directionis  assumed  to  be  inferable  from  hand  data  only.  First,  a  novelsequential  network-based  learning  model  is  developed  for  thesimultaneous  detection  of  hands  and  humans  in  RGB  data.Differing  from  previous  work,  its  performance  is  shown  to  beboth  accurate  and  fast.  Following,  a  new  geometric  method  forestimating  the  3D  pointing  direction  from  depth  data  of  thedetected hand is presented along with a mathematical analysis ofsensor  noise  sensitivity.  Two  new  data  sets  for  pointing  gestureclassification and continuous 3D pointing direction with varyingproximity,  arm  pose  and  background  are  presented.  As  thereare  no  such  data  sets  to  the  best  of  our  knowledge,  both  willbe  publicly  available.  Differing  from  previous  work,  the  robotis  able  to  estimate  the  3D  hand  direction  both  accurately  andfast regardless of hand proximity, background variability or thedetectability  of  specific  human  parts  -  as  demonstrated  by  end-to-end  experimental  results.

## Required Packages
1. OpenCV > 4.4.0
2. PCL library


## Usage
The repository contains source and header files for 3D Pointing Direction Estimation. The code uses RGB and Depth images of the same scene, compare the estimated direction with the ground truth and outputs 3D angle between 2 directions. Here are the instructions to use the code.

1. Download 3D Pointing Direction Data Set and put the folders containing RGB, Depth images and annotation files under the same folder with code files.

2. Download Yolo weights and .cfg files and put them under "yolo_files" folder.

3. Download Pointing Gesture classifier and put it under "classifier_model" folder.

4. Run the code with a c++ compiler.


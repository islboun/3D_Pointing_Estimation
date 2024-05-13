# Detecting 3D Hand Pointing Direction from RGB-D Data in Wide-Ranging HRI Scenarios



This repository contains the codes for our HRI 2022 paper titled as ["Detecting 3D Hand Pointing Direction from RGB-D Data in Wide-Ranging HRI Scenarios".](https://dl.acm.org/doi/abs/10.5555/3523760.3523819) 

[Pointing Gesture](https://bucloud.bogazici.edu.tr/s/ZPws6gP8L33bmpn) & [3D Pointing Direction](https://bucloud.bogazici.edu.tr/s/X6dg3KZ7Ee4Be3o) Data Sets can be downloaded from the links.

## Abstract
This paper addresses the detection of 3D hand point-ing  direction  from  RGB-D  data  by  a  mobile  robot.  Considering ubiquitous forms of pointing gestures, the 3D pointing direction is  assumed  to  be  inferable  from  hand  data  only.  First,  a  novel sequential  network-based  learning  model  is  developed  for  the simultaneous  detection  of  hands  and  humans  in  RGB  data. Differing  from  previous  work,  its  performance  is  shown  to  be both  accurate  and  fast.  Following,  a  new  geometric  method  for estimating  the  3D  pointing  direction  from  depth  data  of  the detected hand is presented along with a mathematical analysis ofsensor  noise  sensitivity.  Two  new  data  sets  for  pointing  gesture classification and continuous 3D pointing direction with varying proximity,  arm  pose  and  background  are  presented.  As  there are  no  such  data  sets  to  the  best  of  our  knowledge,  both  willbe  publicly  available.  Differing  from  previous  work,  the  robotis  able  to  estimate  the  3D  hand  direction  both  accurately  andfast regardless of hand proximity, background variability or the detectability  of  specific  human  parts  -  as  demonstrated  by  end-to-end  experimental  results.

## Required Packages
1. OpenCV > 4.4.0
2. PCL library


## Usage
The repository contains source and header files for 3D Pointing Direction Estimation. The code uses RGB and Depth images of the same scene, compare the estimated direction with the ground truth and outputs 3D angle between 2 directions. Here are the instructions to use the code.

1. Download [3D Pointing Direction Data Set](https://bucloud.bogazici.edu.tr/s/X6dg3KZ7Ee4Be3o) and put the folders containing RGB, Depth images and annotation files under the same folder with code files.

2. Download [Yolo weights and .cfg files](https://bucloud.bogazici.edu.tr/s/GyfjmraaNWPa2ES) and put them under "yolo_files" folder.

3. Download [Pointing Gesture classifier](https://bucloud.bogazici.edu.tr/s/ZPws6gP8L33bmpn) and put it under "classifier_model" folder.

4. Run the code with a c++ compiler.


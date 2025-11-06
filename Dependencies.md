1. Code in the src and include directories

ORBextractor.cc
A modified implementation derived from orb.cpp in the OpenCV library.
The original source code is distributed under the BSD License.

PnPsolver.h and PnPsolver.cc
Adapted from epnp.h and epnp.cc originally developed by Vincent Lepetit.
Equivalent implementations are available in BSD-licensed computer vision frameworks such as OpenCV
 and OpenGV
.
The original code is released under the FreeBSD License.

Function ORBmatcher::DescriptorDistance in ORBmatcher.cc
This function references publicly available bit-counting code from Stanford Bit Hacks
, which is released to the public domain.

2. Code in the Thirdparty directory

DBoW2
Includes a modified version of the DBoW2
 and DLib
 libraries.
Both original projects are distributed under the BSD License.

g2o
Contains a modified version of the g2o
 optimization framework.
All included files are licensed under the BSD License.

3. External library dependencies

Pangolin – Used for visualization and user interface components.
Licensed under the MIT License.

OpenCV – Core computer vision library used for image processing.
Licensed under the BSD License.

Eigen3 – Linear algebra library used for matrix and vector computations.
Versions ≥ 3.1.1 are licensed under MPL2; earlier versions under LGPLv3.

ROS (Optional) – Required only when building the Examples/ROS module.
Licensed under the BSD License.
The manifest declares dependencies on roscpp, tf, sensor_msgs, image_transport, and cv_bridge, all of which are BSD-licensed packages.

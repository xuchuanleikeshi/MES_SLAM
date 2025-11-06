# Overview

This repository provides the implementation of a memory-efficient visual SLAM framework that integrates sparse prior embedding with nonlinear score-guided sparsification. The proposed system is developed upon the classic ORB-SLAM2 architecture, extending its modular design to support adaptive keyframe sparsification and compact map representation. By incorporating prior modeling and information-driven optimization, the framework effectively reduces memory overhead in keyframe and map point management while maintaining high localization and mapping accuracy. The approach is particularly suitable for embedded and resource-constrained platforms, achieving efficient computation and storage without compromising visual SLAM performance.

# Source Code Availability Statement

This repository accompanies the submission to Robotica and provides supplementary materials for review. To protect the authors’ intellectual property and comply with the journal’s peer-review policy, the core source code—including files in the ***src*** and include directories—remains temporarily restricted.
The complete implementation will be released **publicly after the paper is formally accepted by Robotica**, under the declared open-source license. This staged release ensures both research integrity and proper attribution of the proposed SLAM framework.

##  RealSense Stereo Camera Calibration Results

This section presents the **geometric calibration results** of the Intel **RealSense stereo infrared cameras** (`/infra_left` and `/infra_right`) obtained using the **Kalibr** toolchain.  
The calibration target is a **6×6 AprilGrid**, with each tag size of **3 cm** and a spacing of **9 mm**.  
The calibration parameters can be directly used for **stereo matching, depth estimation, and VIO/SLAM initialization**.

---

##  Camera System Parameters

| Camera | Model Type | Distortion Coefficients *(k₁, k₂, k₃, k₄)* | Projection Parameters *(fx, fy, cx, cy)* | Reprojection Error (px) |
|---------|-------------|---------------------------------------------|-------------------------------------------|--------------------------|
| **cam0** `/infra_left` | Equidistant Distorted Pinhole | [0.3136, 0.3246, -0.5265, 0.5851] | [661.50, 662.56, 635.83, 361.51] | ±(0.266, 0.297) |
| **cam1** `/infra_right` | Equidistant Distorted Pinhole | [0.3429, 0.1979, -0.4823, 0.9354] | [629.90, 631.60, 635.52, 353.59] | ±(0.230, 0.258) |

## Kalibr calibrates RealSense D435i binoculars
![image](https://github.com/xuchuanleikeshi/MES_SLAM/blob/main/(Kalibr%20calibrates%20RealSense%20D435i)Screenshot%20from%202025-05-12%2021-21-52.png)<br>




##----------------------------Common Linux commands for system operation----------------------------------
```bash
Tsinghua Mirror Source：pip install -i https://pypi.tuna.tsinghua.edu.cn/simple +Package_name
Exit the virtual environment：conda deactivate
Entering the evo virtual environment：source /home/virtual_env/bin/activate
Graphics card usage：watch -n 1 nvidia-smi
Check storage devices：lsblk
Camera software enabled：realsense-viewer 
See environment variables：echo $PATH
Check and temporarily exclude environment variables.：export PATH=$(echo $PATH | tr ':' '\n' | grep -v 'anaconda' | tr '\n' ':')
Rename the images in the folder：ls *.jpg | awk '{printf("mv \"%s\" \"%06d.png\"\n", $0, NR-1)}' | bash
```
```bash


##----------------------Commonly used visual SLAM commands------------------------------------
Launch file for D435i camera：/opt/ros/noetic/share/realsense2_camera/launch
evo evaluation tool commands：evo_traj   Dataset Name   Data File
tar command to extract to a specified directory：tar -xvf xx.tgz -C  /xx
Dataset format conversion：evo_traj euroc /media/EuRoC/MH_03_medium/mav0/state_groundtruth_estimate0/data.csv --save_as_tum
Convert .txt files into TUM format track files.:sed 's/\.000000/e-09/g' input.txt > output.txt
Euroc dataset comparison experiment：
evo_traj tum /home/MES-SLAM-master/output.txt  -p  --ref=data.tum --plot_mode xyz -a --correct_scale
evo_traj tum /home//MES-SLAM-master/MH05_output.txt -p --ref=data.tum --plot_mode xyz -a --correct_scale 
evo_ape tum data.tum /home/MES-SLAM-master/MH05_output.txt -vas --plot
```
		     
		     
			     
##Kitti dataset MES-SLAM commands on the CLion platform:
```bash
00 dataset：/home/MES-SLAM-master/Vocabulary/ORBvoc.txt /home/MES-SLAM-master/Examples/Stereo/KITTI00-02.yaml /media/NewDisk/Kitti00-10/00
04 dataset：/home/MES-SLAM-master/Vocabulary/ORBvoc.txt /home/MES-SLAM-master/Examples/Stereo/KITTI04-12.yaml /media/NewDisk/Kitti00-10/0	     
Kitti Dataset comparison experiment：evo_traj kitti  /home/EMS-SLAM-master/CameraTrajectory.txt  --ref=04.txt -p   --plot_mode xz
```


------------------------------------------------------------------------------
```bash
EMS-SLAM command：1):cd kalibr_ws  2):source ./devel/setup.bash 
Euroc dataset: Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml  /media/EuRoC/MH_03_medium Examples/Stereo/EuRoC_TimeStamps/MH03.txt

```

```bash
Kitti dataset：Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI04-12.yaml /media/NewDisk/Kitti00-10/04
Tum dataset：./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM3.yaml /media/NewDisk/TUM/rgbd_dataset_freiburg3_walking_xyz /media/TUM/rgbd_dataset_freiburg3_walking_xyz/associations.txt

Launch RealSense camera：roslaunch realsense2_camera rs_camera.launch
roslaunch imu_utils D435i_imuCali.launch
rosbag play -r 400 D435i_imu.bag
```

Kalibr calibration test：  rosrun kalibr kalibr_calibrate_cameras --bag /home/cam_april.bag --topics /cam0/image_raw /cam1/image_raw --models pinhole-radtan pinhole-radtan --target /home/Download/april_6x6.yaml









----------------------------liunx常见指令----------------------------------
清华镜像源：pip install -i https://pypi.tuna.tsinghua.edu.cn/simple +包名
退出虚拟环境：conda deactivate
进入evo虚拟环境：source /home/xulei/virtual_env/bin/activate
显卡使用情况：watch -n 1 nvidia-smi
linux自带录制屏幕工具：Ctrl + Shift + Alt + R
查看电脑的外界设备：dmesg | grep -i usb
区域截图到粘贴板快捷键：shift + ctrl + prtsc
查看存储设备：lsblk
相机软件开启：realsense-viewer 
参看环境变量：echo $PATH
查看并临时排除环境变量：export PATH=$(echo $PATH | tr ':' '\n' | grep -v 'anaconda' | tr '\n' ':')
文件夹下的图片重新命名：ls *.jpg | awk '{printf("mv \"%s\" \"%06d.png\"\n", $0, NR-1)}' | bash

----------------------视觉slam常用指令------------------------------------
D435i相机的launch文件：/opt/ros/noetic/share/realsense2_camera/launch
evo评估工具命令：evo_traj 数据集名称 数据文件
tar解压到指定目录指令：tar -xvf xx.tgz -C  /xx
数据集格式转换：evo_traj euroc /media/xulei/新加卷/EuRoC/MH_03_medium/mav0/state_groundtruth_estimate0/data.csv --save_as_tum
.txt转化成符合TUM 形式的轨迹文件格式:sed 's/\.000000/e-09/g' input.txt > output.txt
Euroc数据集对比实验：evo_traj tum /home/xulei/MS-SLAM-master/output.txt  -p  --ref=data.tum --plot_mode xyz -a --correct_scale
		     evo_traj tum /home/xulei/MS-SLAM-master/MH05_output.txt -p --ref=data.tum --plot_mode xyz -a --correct_scale 
		     evo_ape tum data.tum /home/xulei/MS-SLAM-master/MH05_output.txt -vas --plot
		     
		     
		     
		     
Kitti数据集MS-SLAM在clion端命令：
00数据集：/home/xulei/MS-SLAM-master/Vocabulary/ORBvoc.txt /home/xulei/MS-SLAM-master/Examples/Stereo/KITTI00-02.yaml /media/xulei/新加卷/Kitti00-10/00
04数据集：/home/xulei/MS-SLAM-master/Vocabulary/ORBvoc.txt /home/xulei/MS-SLAM-master/Examples/Stereo/KITTI04-12.yaml /media/xulei/新加卷/Kitti00-10/0	     
Kitti数据集对比试验：evo_traj kitti  /home/xulei/MS-SLAM-master/CameraTrajectory.txt  --ref=04.txt -p   --plot_mode xz


------------------------------------------------------------------------------
MS-slam运行命令：1):cd kalibr_ws  2):source ./devel/setup.bash 
Euroc数据集: Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml  /media/xulei/新加卷/EuRoC/MH_03_medium Examples/Stereo/EuRoC_TimeStamps/MH03.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/MH_01_easy Examples/Stereo/EuRoC_TimeStamps/MH01.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/MH_02_easy Examples/Stereo/EuRoC_TimeStamps/MH02.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/MH_03_medium Examples/Stereo/EuRoC_TimeStamps/MH03.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/MH_04_difficult Examples/Stereo/EuRoC_TimeStamps/MH04.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/MH_05_difficult Examples/Stereo/EuRoC_TimeStamps/MH05.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V1_01_easy Examples/Stereo/EuRoC_TimeStamps/V101.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V1_02_medium Examples/Stereo/EuRoC_TimeStamps/V102.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V1_03_difficult Examples/Stereo/EuRoC_TimeStamps/V103.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V2_01_easy Examples/Stereo/EuRoC_TimeStamps/V201.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V2_02_medium Examples/Stereo/EuRoC_TimeStamps/V202.txt
Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /media/xulei/新加卷/EuRoC/V2_03_difficult Examples/Stereo/EuRoC_TimeStamps/V203.txt 



Kitti数据集：Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI04-12.yaml /media/xulei/新加卷/Kitti00-10/04
Tum数据集：./Examples/RGB-D/rgbd_tum ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/TUM3.yaml /media/xulei/新加卷/TUM/rgbd_dataset_freiburg3_walking_xyz /media/xulei/新加卷/TUM/rgbd_dataset_freiburg3_walking_xyz/associations.txt

启动 RealSense 相机：roslaunch realsense2_camera rs_camera.launch
录制 IMU 数据：rosbag record -O D435i_imu /camera/imu
roslaunch imu_utils D435i_imuCali.launch
rosbag play -r 400 D435i_imu.bag



kalibr运行初始化环境变量：cd 
kalibr标定测试：  rosrun kalibr kalibr_calibrate_cameras --bag /home/xulei/下载/cam_april.bag --topics /cam0/image_raw /cam1/image_raw --models pinhole-radtan pinhole-radtan --target /home/xulei/下载/april_6x6.yaml




resultFile = "camchain-" + bagtag + ".yaml"
resultFileTxt = "results-cam-" + bagtag + ".txt"
reportFile = "report-cam-" + bagtag + ".pdf"


cp -r /home/xulei/firmware.bak/* /lib/firmware/

-- Glog Include Directories: /usr/include
-- Glog Libraries: /usr/lib/x86_64-linux-gnu/libglog.so
-- Glog Found: TRUE
# 确保 CMake 忽略 Anaconda 目录
# 忽略 Anaconda 目录，避免错误包含 glog
set(CMAKE_IGNORE_PATH "/opt/anaconda3/include" "/opt/anaconda3/lib")

# 强制使用系统安装的 glog
set(GLOG_INCLUDE_DIR "/usr/include")
set(GLOG_LIBRARY "/usr/lib/x86_64-linux-gnu/libglog.so")

include_directories(BEFORE SYSTEM ${GLOG_INCLUDE_DIR})
link_directories("/usr/lib/x86_64-linux-gnu")



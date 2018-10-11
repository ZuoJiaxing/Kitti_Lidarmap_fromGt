
**此程序用来把kitti的激光点云根据groundtru拼接到第一帧矫正后的灰度左相机坐标系下。

首先改cpp文件中的数据集的文件目录, 然后:
$ ./constructLaserMap 05/velodyne/  05.txt 05Totalmap.pcd

_________________________________________________________________

!!!一定要注意外参Tr_velo_to_cam是不是对应相应的序列. 
./constructLaserMap 05/velodyne/  05.txt
./constructLaserMap 04/velodyne/  04.txt
./constructLaserMap 01/velodyne/  01.txt
./constructLaserMap 06/velodyne/  06.txt
运行程序前，要在可执行程序目录下创建generatePCD文件夹


在存储激光点云时，为了和opencv中的坐标轴保持一致，我把激光点的y加了一个负号.

#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>  // for formating strings
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <vector>
#include <string>
// #include <regex>
// #include <ctime>
// #include <cstdlib>
#include <sstream>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iterator>
#include <math.h>


using namespace std;
using namespace pcl;

#define DISTHRE 50.0

int loadPoses(string file_name, vector<Eigen::Matrix4d> &poses) ;
void readLaserData (string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre);
void readLaserData(string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre, Eigen::Matrix4d &transform);

int main( int argc, char** argv )
{

    if (argc != 3)
    {
        //example: ./project        05/velodyne/  05.txt
        cerr << endl << "Usage: ./project    xx/velodyne/      xx.txt " << endl;
        // ros::shutdown();
        return 1;
    }

    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Matrix4d> poses;         // 相机位姿

    string prePathToLaserSequence = "/home/zuojiaxing/Documents/dataset/Kitti/originalOdometryDataset/velodyne/" + std::string(argv[1]); //laser数据
    string PathToGroundtru = "/home/zuojiaxing/Documents/dataset/Kitti/originalOdometryDataset/data_odometry_poses/" + std::string(argv[2]);  //poses文件
    cout << prePathToLaserSequence << endl;
    cout << PathToGroundtru << endl;
    //标定的外参
    Eigen::Matrix4d Tr_velo_to_cam;
       Tr_velo_to_cam  << -1.857739385241e-03,  -9.999659513510e-01,  -8.039975204516e-03,  -4.784029760483e-03,
 -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
  9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
  0,0,0,1;///从data_odometry_calib/05/calib.txt中读取的Tr( from velo to rectC0)


    static Eigen::Matrix4d tmpPose;
    static string laserfilename;

    //load  poses
    if (loadPoses(PathToGroundtru,  poses) == 0 )
    {
        cerr << "cannot find pose file" << endl;
        return 1;
    }


    // 计算点云并拼接
    // 相机内参
    // double cx = 325.5;
    // double cy = 253.5;
    // double fx = 518.0;
    // double fy = 519.0;
    // double depthScale = 1000.0;

    cout << "正在将laser数据转换为PCL点云..." << endl;

    // 定义点云使用的格式：这里用的是PointXYZI;并新建点云用来存储总的地图点云;TmptotalMappoints 用来保存50帧的数据点云，方便做滤波
    pcl::PointCloud<PointXYZI>::Ptr  totalMappoints (new pcl::PointCloud<PointXYZI>);
    pcl::PointCloud<PointXYZI>::Ptr  TmptotalMappoints (new pcl::PointCloud<PointXYZI>);
    cout << "test" << "poses.size: " << poses.size() << endl;

    int j = 0;
    int k = 0;
    for ( int i = 0; i < 120; i++ ) //ADJU    for ( int i = 0; i < poses.size(); i++ )
    {

        cout << "转换laser数据中: " << i  << endl;
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        laserfilename = prePathToLaserSequence + ss.str() + ".bin";
        cout << laserfilename << endl;
        tmpPose = poses[i] * Tr_velo_to_cam; // 这里的poses为 第一个矫正后的灰度左相机下的from ck to c0
       
        //pcl::PointCloud<PointXYZI>::Ptr  framepoints (new pcl::PointCloud<PointXYZI>);//单帧的激光点云
        pcl::PointCloud<PointXYZI>::Ptr  Trans_framepoints (new pcl::PointCloud<PointXYZI>);//转移到相机2坐标系（开始时刻）下的单帧的激光点云

        readLaserData(laserfilename, Trans_framepoints, DISTHRE, tmpPose);
        // pcl::io::savePCDFileBinary("Trans_framePointsmap.pcd", *Trans_framepoints );
        // depth filter and statistical removal
        pcl::PointCloud<PointXYZI>::Ptr tmpPointCloud ( new pcl::PointCloud<PointXYZI> );
        pcl::StatisticalOutlierRemoval<PointXYZI> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(Trans_framepoints );
        statistical_filter.filter( *tmpPointCloud );
        (*TmptotalMappoints) += *tmpPointCloud;
        j++;
        if (j == 20 )
        {
            // voxel filter
            pcl::VoxelGrid<PointXYZI> voxel_filter;
            //TODO  当分辨率设置低于0.01时会显示[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
            voxel_filter.setLeafSize( 0.03, 0.03, 0.03 );       // resolution 
            //voxel_filter.setLeafSize( 0.05, 0.05, 0.05 );       // resolution
            pcl::PointCloud<PointXYZI>::Ptr tmpPointCloud1 ( new pcl::PointCloud<PointXYZI> );
            voxel_filter.setInputCloud( TmptotalMappoints );
            voxel_filter.filter( *tmpPointCloud1 );
            tmpPointCloud1->swap( *TmptotalMappoints );
            // (*totalMappoints) += *TmptotalMappoints;
            // 每20帧保存为一个pcd文件
            stringstream sspcd;
            sspcd << setfill('0') << setw(6) << k;
            string  Partpcdfile = "generatePCD/part" + sspcd.str() + ".pcd";
            cout << "Now save the " << Partpcdfile << endl;
            pcl::io::savePCDFileBinary(Partpcdfile, *TmptotalMappoints );
            (*TmptotalMappoints).clear();
            (*tmpPointCloud1).clear();
            j = 0;
            k++;
        }

    }

    //totalMappoints->is_dense = false;
    //cout << "点云共有" << totalMappoints->size() << "个点." << endl;

    // // voxel filter
    // pcl::VoxelGrid<PointXYZI> voxel_filter;
    // voxel_filter.setLeafSize( 0.01, 0.01, 0.01 );       // resolution
    // pcl::PointCloud<PointXYZI>::Ptr tmpPointCloud2 ( new pcl::PointCloud<PointXYZI> );
    // voxel_filter.setInputCloud( totalMappoints );
    // voxel_filter.filter( *tmpPointCloud2 );
    // tmpPointCloud2->swap( *totalMappoints );
    //    cout << "滤波之后，点云共有" << totalMappoints->size() << "个点." << endl;
    //
    //cout << "Now save the PointCloud to .pcd file."
    //     pcl::io::savePCDFileBinary("Totalmap.pcd", *totalMappoints );
    return 0;
}




int loadPoses(string file_name, vector<Eigen::Matrix4d> & poses) {
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
        return  0;
    while (!feof(fp)) {
        double P[3] [4];
        if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P[0][0], &P[0][1], &P[0][2], &P[0][3],
                   &P[1][0], &P[1][1], &P[1][2], &P[1][3],
                   &P[2][0], &P[2][1], &P[2][2], &P[2][3] ) == 12) {
            Eigen::Matrix4d T;
            T << P[0][0], P[0][1], P[0][2], P[0][3],
            P[1][0], P[1][1], P[1][2], P[1][3],
            P[2][0], P[2][1], P[2][2], P[2][3],
            0, 0, 0, 1;
            poses.push_back(T);
        }
    }
    fclose(fp);
    return 1;

}


// void readLaserData(string &filename, Eigen::Vector3d & Position, Eigen::Vector3d &RGB)
// {
//     double *variable = new double;
//     ifstream  fin(filename, "rb");
//     if (fin)
//     {
//         cerr<<"TODO readLaserData! "<<endl;
//         //参考   /home/zuojiaxing/Documents/dataset/kittool/kitti-pcl/KITTI_README.TXT
//         //或者参考　　/home/zuojiaxing/Documents/dataset/kittool/到 OTHERS 的链接/devkit/readme.txt　（和上面的文件是一样的）


//     } else {
//         cout << "readLaserData ERROR:" << endl;
//     }
// }


//读入单帧点云数据到points  // refer to kitti-pcl
void readLaserData(string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre)
{
    // load point cloud
    fstream input(filename, ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Could not read  laser file: " << filename << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++)
    {
        PointXYZI point;
        // input.read((char *) &point.x, 3 * sizeof(float)); //这也是对的，&point.x 是取地址
        input.read((char *) &point.x,  sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
	
        // 这里也可以过滤掉距离当前位置比较远的点和距离特别近的点
        double dis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (dis > disThre | dis < 1.0)
            continue;

		//NOTE 为了和opencv中的y轴方向保持一致
	//cout<<"before:	"<<point.y<<endl;
	point.y=-point.y;
	//cout<<"after:	"<<point.y<<endl;

        points->push_back(point);
    }
    input.close();
}

//TODO
//读入单帧点云数据并经过刚体T变换到points， points=T*原始的激光点云数据    //默认传入的参数为 from  velo to cam
void readLaserData(string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre, Eigen::Matrix4d &transform)
{
    // load point cloud
    fstream input(filename, ios::in | ios::binary);
    if (!input.good()) {
        cerr << "Could not read  laser file: " << filename << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);

    for (int i = 0; input.good() && !input.eof(); i++)
    {
        PointXYZI point;
        // input.read((char *) &point.x, 3 * sizeof(float));
        input.read((char *) &point.x,  sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));


        // 这里也可以过滤掉距离当前位置(velo)比较远的点和距离特别近的点
        double dis = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (dis > disThre | dis < 1.0)
            continue;
        Eigen::Matrix<double, 4, 1> ptPosition41, ptTransPosition41;
        ptPosition41 << point.x, point.y, point.z, 1.0;
        ptTransPosition41 = transform * ptPosition41;
        point.x = ptTransPosition41(0, 0);
        point.y = ptTransPosition41(1, 0);
        point.z = ptTransPosition41(2, 0);

	//NOTE 为了和opencv中的y轴方向保持一致
	//cout<<"before:	"<<point.y<<endl;
	point.y=-point.y;
	//cout<<"after:	"<<point.y<<endl;

        points->push_back(point);
    }
    input.close();
}

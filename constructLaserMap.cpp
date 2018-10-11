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
#include <sstream>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <iterator>
#include <math.h>


using namespace std;
using namespace pcl;

#define DISTHRE 40.0
#define RESULOTION 0.2

int loadPoses(string file_name, vector<Eigen::Matrix4d> &poses) ;
void readLaserData (string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre);
void readLaserData(string &filename, pcl::PointCloud<PointXYZI>::Ptr &points, double disThre, Eigen::Matrix4d &transform);

int main( int argc, char** argv )
{

    if (argc != 5)
    {
        //example: ./project        velodyne/  05.txt  savepcd.pcd  100
        cerr << endl << "Usage: ./project    xx/velodyne/  xx.txt  savepcd.pcd   nframes" << endl;
        // ros::shutdown();
        return 1;
    }

    vector<cv::Mat> colorImgs, depthImgs;    // 彩色图和深度图
    vector<Eigen::Matrix4d> poses;         // 相机位姿

    string prePathToLaserSequence = "/home/rocky/Documents/dataset/kitti/05_velodyne/" + std::string(argv[1]); //laser数据
    string PathToGroundtru = "/home/rocky/Documents/dataset/kitti/Kittiodometry/dataset_poses/poses/" + std::string(argv[2]);  //poses文件
    cout << prePathToLaserSequence << endl;
    cout << PathToGroundtru << endl;

    //标定的外参
    Eigen::Matrix4d Tr_velo_to_cam;
    Tr_velo_to_cam  << -1.857739385241e-03,  -9.999659513510e-01,  -8.039975204516e-03,  -4.784029760483e-03,
            -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
            9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
            0,0,0,1;///从data_odometry_calib/05/calib.txt中读取的Tr( from velo to rectC0) 04-12

    // Tr_velo_to_cam  <<4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
    // -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
    // 9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
    // 0,0,0,1;///从data_odometry_calib/01/calib.txt中读取的Tr( from velo to rectC0)//00-02sequence

    //    Tr_velo_to_cam  <<2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02, -2.796816941295e-03,
    //            1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01, -7.510879138296e-02,
    //            9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02, -2.721327964059e-01,
    //            0, 0, 0, 1;//从data_odometry_calib/03/calib.txt中读取的Tr( from velo to rectC0)//03sequence

    static Eigen::Matrix4d tmpPose;
    static string laserfilename;

    // load  poses
    if (loadPoses(PathToGroundtru,  poses) == 0 )
    {
        cerr << "cannot find pose file" << endl;
        return 1;
    }

    cout << "正在将laser数据转换为PCL点云..." << endl;

    // 定义点云使用的格式：这里用的是PointXYZI;并新建点云用来存储总的地图点云;TmptotalMappoints 用来保存50帧的数据点云，方便做滤波
    pcl::PointCloud<PointXYZI>::Ptr  totalMappoints (new pcl::PointCloud<PointXYZI>);// Whole map
    pcl::PointCloud<PointXYZI>::Ptr  localMappoints (new pcl::PointCloud<PointXYZI>);// Local map includes only ~=20 frmaes.
    cout << "test" << "poses.size: " << poses.size() << endl;

    int nframes = std::atoi(argv[4]);
    cout<<nframes<<" frames will be processed!"<<endl;
    int j = 0;
    int k = 0;
    for ( int i = 0; i < nframes; i++ ) //ADJU    for ( int i = 0; i < poses.size(); i++ )
    {

        cout << "转换laser数据中: " << i  << endl;
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        laserfilename = prePathToLaserSequence + ss.str() + ".bin";
        cout << laserfilename << endl;
        tmpPose = poses[i] * Tr_velo_to_cam; // 这里的poses为 第一个矫正后的灰度左相机下的from ck to c0
        pcl::PointCloud<PointXYZI>::Ptr  Trans_framepoints (new pcl::PointCloud<PointXYZI>);//转移到相机2坐标系（开始时刻）下的单帧的激光点云
        readLaserData(laserfilename, Trans_framepoints, DISTHRE, tmpPose);


        /// Depth filter and statistical removal for single frame
#if 0
        // Voxel filter
        pcl::VoxelGrid<PointXYZI> voxel_filter;
        //TODO  当分辨率设置低于0.01时会显示[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
        voxel_filter.setLeafSize( RESULOTION,RESULOTION, RESULOTION );       // resolution
        voxel_filter.setInputCloud( Trans_framepoints );
        voxel_filter.filter( *Trans_framepoints );


        // Statistical_filter
        pcl::StatisticalOutlierRemoval<PointXYZI> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(Trans_framepoints );
        statistical_filter.filter( *Trans_framepoints );
#endif

        (*localMappoints) += *Trans_framepoints;

        ///每隔20帧filter一次

        j++;
        if (j == 20 )
        {
            // Voxel filter
            pcl::VoxelGrid<PointXYZI> voxel_filter;
            //TODO  当分辨率设置低于0.01时会显示[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
            voxel_filter.setLeafSize( RESULOTION,RESULOTION, RESULOTION );       // resolution
            voxel_filter.setInputCloud( localMappoints );
            voxel_filter.filter( *localMappoints );


            // Statistical_filter
            pcl::StatisticalOutlierRemoval<PointXYZI> statistical_filter;
            statistical_filter.setMeanK(50);
            statistical_filter.setStddevMulThresh(1.0);
            statistical_filter.setInputCloud(localMappoints );
            statistical_filter.filter( *localMappoints );

            //            // 每20帧保存为一个pcd文件
            //            stringstream sspcd;
            //            sspcd << setfill('0') << setw(6) << k;
            //            string  Partpcdfile = "generatePCD/part" + sspcd.str() + ".pcd";
            //            cout << "Now save the " << Partpcdfile << endl;
            //            pcl::io::savePCDFileBinary(Partpcdfile, *localMappoints );

            j = 0;
            k++;

            // Add the localMappoints to totalMappoints
            (*totalMappoints) += *localMappoints;

        }


    }


    /// voxel filter for the totalMappoints. NOTE: If the map is large and the RESULOTION is high, just skip this.
    pcl::VoxelGrid<PointXYZI> voxel_filter;
    voxel_filter.setLeafSize( RESULOTION, RESULOTION, RESULOTION );       // resolution
    voxel_filter.setInputCloud( totalMappoints );
    voxel_filter.filter( *totalMappoints );

    totalMappoints->is_dense = false;
    cout << "点云共有" << totalMappoints->size() << "个点." << endl;
    cout << "Now save the PointCloud to .pcd file: "<<std::string(argv[3])<<endl;
    pcl::io::savePCDFileBinary(std::string(argv[3]), *totalMappoints );
    cout<<"---------------save done!----------------"<<endl;
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

        ///NOTE 为了和opencv中的y轴方向保持一致
        //point.y=-point.y;//NOTE
        points->push_back(point);
    }
    input.close();
}

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
        //point.y=-point.y; //NOTE
        points->push_back(point);
    }
    input.close();
}

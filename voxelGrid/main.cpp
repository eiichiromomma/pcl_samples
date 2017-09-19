#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (argc < 2){
        std::cout << "Usage: voxelGrid pcdfile.pcd" << std::endl;
    }else{
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1){
            return -1;
        }
    }

    //入力、出力、処理全てについてテンプレートの方が一致している必要がある(ここではpcl::PointXYZ)
    pcl::VoxelGrid<pcl::PointXYZ> vog;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vogCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vog.setInputCloud(cloud);
    // x, y, zのVoxelサイズを決定する
    vog.setLeafSize(1.0, 0.5, 1.0);
    vog.filter(*vogCloud);

    //作成したいviewportの数だけ定義する
    int v1(0), v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    viewer->initCameraParameters();
    //ウィンドウは(0,0)-(1.0,1.0)として考え、これにviewportを割り当てる
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setSize(800, 800);
    //viewerにおける各処理は名令＋データの名称＋ビューポートの組み合わせで記述する
    //データの変数名と名称は一致させる必要はない
    //v1ビューポートの"Point Cloud"への処理
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud", v1);
    //ビューポートを省略した場合は全ビューポートが対象になる。全ビューポートの”vogCloud"への処理
    viewer->addPointCloud<pcl::PointXYZ>(vogCloud, "vogCloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "vogCloud");
    //ビューポートを絞ってもデータの名称が優先されて全ビューポートで色が変わる
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.5, "vogCloud", v2);

    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

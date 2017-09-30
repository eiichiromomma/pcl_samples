#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/visualization/cloud_viewer.h>

#include <QApplication>
#include <QFileDialog>
#include <QString>
#include <QWidget>
#include <QDebug>

int main(int argc, char** argv)
{
    QApplication a(argc, argv);
    QFile file;
    auto filename = QFileDialog::getOpenFileName();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (filename.isNull()){
        std::cout << "Usage: pcdViewer pcdfile.pcd" << std::endl;
    }else{
        if (pcl::io::loadPCDFile(filename.toStdString(), *cloud) == -1){
            return -1;
        }
    }
    //全点群のBouding boxが取れる
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    std::cout << "Min: " << min_pt << std::endl;
    std::cout << "Max: " << max_pt << std::endl;

    int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    viewer->initCameraParameters();
    viewer->setSize(800, 800);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud", v1);
    viewer->addCoordinateSystem(1.0);

    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

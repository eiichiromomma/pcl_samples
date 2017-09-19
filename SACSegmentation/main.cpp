#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (argc < 2){
        std::cout << "Usage: SACSegmentation pcdfile.pcd" << std::endl;
    }else{
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1){
            return -1;
        }
    }

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //SACSegmentationの出力はIndices(PointCloudの何番目のデータかの羅列)とCoefficients(方程式の係数)
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
    seg.setInputCloud(cloud);
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    //PLANEだとシンプルな面しか指定できないがPERPENDICULAR_PLANEだとマージンを指定できる(必須)
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    //法線の指定
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    seg.setAxis(axis);
    //例えば法線から10度の勾配マージンを許す
    seg.setEpsAngle(M_PI * 10.0/180.0);
    //モデルからの外れ値の許容範囲
    seg.setDistanceThreshold(0.3);
    seg.segment(*inliers, *planeCoeff);
    if (inliers->indices.size() == 0){
        std::cout << "Not found" << std::endl;
        return -1;
    }
    //Coefficientsの表示
    std::cout << "Plane Coefficients: " << *planeCoeff << std::endl;

    //cloudのinliersを抽出する
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ext.setInputCloud(cloud);
    ext.setIndices(inliers);
    //trueにするとinliers以外を抽出できる
    ext.setNegative(false);
    ext.filter(*planeCloud);

    int v1(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 5, 1, 0, 1);
    viewer->setSize(800, 800);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "Point Cloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(planeCloud, "planeCloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "planeCloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1.0, 0.3, "planeCloud", v1);


    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

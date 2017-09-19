#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (argc < 2){
        std::cout << "Usage: pcdViewer pcdfile.pcd" << std::endl;
    }else{
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1){
            return -1;
        }
    }

    //VoxelGridで間引いてからSACSegmentation
    pcl::VoxelGrid<pcl::PointXYZ> vog;
    pcl::PointCloud<pcl::PointXYZ>::Ptr vogCloud(new pcl::PointCloud<pcl::PointXYZ>);
    vog.setInputCloud(cloud);
    // x, y, zのVoxelサイズを決定する
    vog.setLeafSize(0.5, 0.5, 0.2);
    vog.filter(*vogCloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //SACSegmentationの出力はIndices(PointCloudの何番目のデータかの羅列)とCoefficients(方程式の係数)
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
    seg.setInputCloud(vogCloud);
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ext.setInputCloud(vogCloud);
    ext.setIndices(inliers);
    //trueにするとinliers以外を抽出できる
    ext.setNegative(true);
    ext.filter(*outlierCloud);

    //統計的にまとまりのないOutlierを除去する
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    sor.setInputCloud(outlierCloud);
    //何点以上の塊か
    sor.setMeanK(20);
    //どの程度のばらつきを許容するか
    sor.setStddevMulThresh(0.3);
    sor.filter(*sorCloud);

    //近傍点との距離に基づいたOutlierの除去
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ror.setInputCloud(outlierCloud);
    ror.setRadiusSearch(1.0);
    ror.setMinNeighborsInRadius(20);
    ror.filter(*rorCloud);

    int v1(0), v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    //ウィンドウは(0,0)-(1.0,1.0)として考え、これにviewportを割り当てる
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 5, 1, 0, 1);
    viewer->setSize(800, 800);
    viewer->addPointCloud<pcl::PointXYZ>(vogCloud, "Point Cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 0.3, 0.3, "Point Cloud");
    viewer->addPointCloud<pcl::PointXYZ>(outlierCloud, "outlierCloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "outlierCloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.3, 0.3, "outlierCloud");
    viewer->addPointCloud<pcl::PointXYZ>(sorCloud, "objCloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objCloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1.0, 0.3, "objCloud", v1);
    viewer->addPointCloud<pcl::PointXYZ>(rorCloud, "objCloudror", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objCloudror", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1.0, 0.3, "objCloudror", v2);




    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    if (argc < 2){
        std::cout << "Usage: tranICPRegistration pcd_folder" << std::endl;
    }
    const fs::path data_path(argv[1]);
    fs::directory_iterator end;
    std::vector<fs::path> paths;
    if (fs::is_directory(data_path)){
        for(fs::directory_iterator it(data_path); it != end; it++){
            if(!fs::is_directory(it->path()) && it->path().extension() == ".pcd"){
                paths.push_back(it->path());
            }
        }
    }else if(!fs::is_directory(data_path) && data_path.extension() == ".pcd"){
        paths.push_back(data_path);
    }
    if (paths.size() == 0){
        std::cout << "Not Found" << std::endl;
        return -1;
    }
    //registrationの結果を蓄積するPointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr regCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //最初の位置のPointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr iniCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //単純に蓄積しするPointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr accCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader pr;
    pcl::console::TicToc time;
    //設定等はループ外に置いて，input, target等を随時変えれば使い回せる
    //路面(平面)の抽出
    pcl::SACSegmentation<pcl::PointXYZ> seg;
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
    //cloudのinliersを抽出する
    pcl::ExtractIndices<pcl::PointXYZ> ext;
    //trueにするとinliers以外を抽出できる
    ext.setNegative(true);
    //統計的にまとまりのないOutlierを除去する
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    //何点以上の塊か
    sor.setMeanK(20);
    //どの程度のばらつきを許容するか
    sor.setStddevMulThresh(0.3);
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    //icpRegistrationより遥かに狭い範囲で済む
    icp.setMaxCorrespondenceDistance(4.0);
    //iterationの回数(条件1)
    icp.setMaximumIterations(500);
    //条件2
    icp.setTransformationEpsilon(1e-10);
    //条件3
    icp.setEuclideanFitnessEpsilon(1e-12);
    //ICPの結果である行列を保存する。最初は何もしない
    Eigen::Matrix4f icp_transformation = Eigen::Matrix4f::Identity();

    for (unsigned int nfile = 0; nfile < paths.size(); nfile++) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pr.read(paths.at(nfile).string(), *cloud);
        std::cout << paths.at(nfile).string() << std::endl;
        //SACSegmentationの出力はIndices(PointCloudの何番目のデータかの羅列)とCoefficients(方程式の係数)
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *planeCoeff);
        if (inliers->indices.size() == 0){
            std::cout << "Not found" << std::endl;
            return -1;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
        ext.setInputCloud(cloud);
        ext.setIndices(inliers);
        ext.filter(*outlierCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr sorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        sor.setInputCloud(outlierCloud);
        sor.filter(*sorCloud);

        if (nfile == 0){
            pcl::copyPointCloud(*sorCloud, *regCloud);
            pcl::copyPointCloud(*sorCloud, *iniCloud);
            pcl::copyPointCloud(*sorCloud, *accCloud);
            continue;
        }
        //Pointcloudの加算
        *accCloud += *sorCloud;
        time.tic();
        //前のループのICPの結果を反映させる(同じ速度で移動していればだいたいいい場所に移動できる
        pcl::transformPointCloud(*sorCloud, *sorCloud, icp_transformation);
        icp.setInputSource(sorCloud);
        icp.setInputTarget(regCloud);
        icp.align(*sorCloud);
        //行列を取得
        icp_transformation = icp.getFinalTransformation();
        //ICP後のPointcloudの加算
        *regCloud += *sorCloud;

        std::cout << "ICP Score: " << icp.getFitnessScore() << std::endl;
        std::cout << "ICP: " << time.toc() << " ms" << std::endl;

    }
    int v1(0), v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 5, 1, 0, 1);
    viewer->addCoordinateSystem(1.0);
    viewer->setSize(800, 800);

    viewer->addPointCloud<pcl::PointXYZ>(regCloud, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.9, "Point Cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "Point Cloud", v1);

    viewer->addPointCloud<pcl::PointXYZ>(accCloud, "acc", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "acc", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.9, 0.9, "acc", v2);

    viewer->addPointCloud<pcl::PointXYZ>(iniCloud, "ini");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ini");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.2, 0.2, "ini");

    pcl::PCDWriter pcw;
    pcw.write("regCloud.pcd", *regCloud, false);
    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

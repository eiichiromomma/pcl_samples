#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (argc < 2){
        std::cout << "Usage: regionGrowing pcdfile.pcd" << std::endl;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr outlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ext.setInputCloud(cloud);
    ext.setIndices(inliers);
    //trueにするとinliers以外を抽出できる
    ext.setNegative(true);
    ext.filter(*outlierCloud);

    //近傍点との距離に基づいたOutlierの除去
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
    pcl::PointCloud<pcl::PointXYZ>::Ptr rorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ror.setInputCloud(outlierCloud);
    ror.setRadiusSearch(1.0);
    ror.setMinNeighborsInRadius(20);
    ror.filter(*rorCloud);

    //法線の推定
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    //KDtreeが必要。Ptrにするべきか否かはTutorialsでもまちまちで決まりは不明
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    ne.setSearchMethod(tree);
    //法線推定をする範囲。大きいほど大雑把な推定になる。小さすぎると過剰にぶれる
    ne.setRadiusSearch(0.5);
    //裏表を決める視点(ここでは上空)
    ne.setViewPoint(0, 0, 10);
    ne.setInputCloud(rorCloud);
    ne.compute(*cloudNormals);

    //RegionGrowingでクラスタリング
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    //クラスタとして認めるか否かのしきい値
    reg.setMinClusterSize (30);
    reg.setMaxClusterSize (10000);
    reg.setSearchMethod (tree);
    //大きいほど大きい塊になる
    reg.setNumberOfNeighbours (100);
    reg.setInputCloud (rorCloud);
    reg.setInputNormals (cloudNormals);
    //表面の粗さや法線で制限する場合
    //reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    //reg.setCurvatureThreshold (0.5);
    //extractで抽出されるのはPointIndicesのvector。PointIndicesを使った抽出も可能
    std::vector <pcl::PointIndices> clusters;
    reg.extract(clusters);
    //色付きの点群だけ欲しい場合
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud = reg.getColoredCloud();

    int v1(0), v2(0);
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud"));
    //ウィンドウは(0,0)-(1.0,1.0)として考え、これにviewportを割り当てる
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 5, 1, 0, 1);
    viewer->setSize(800, 800);
    viewer->addPointCloud<pcl::PointXYZ>(rorCloud, "objCloudror", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "objCloudror", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.3, 1.0, 0.3, "objCloudror", v1);
    //法線の可視化にはペアとなるXYZを持つ点群を一緒に渡すか、PointNormalを作る
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(rorCloud, cloudNormals, 2, 0.5f, "cnormal", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.8, "cnormal", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cnormal", v1);

    //色付きの点群を表示する
    viewer->addPointCloud<pcl::PointXYZRGB>(coloredCloud, "coloredc", v2);

    while(!viewer->wasStopped()){
        viewer->spinOnce(10);
    }
    return 0;
}

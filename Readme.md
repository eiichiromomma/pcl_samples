PCLのサンプル
---

1. pcdViewer: PCLVisualizerを使ったpcdファイルの表示
2. voxelGrid: Voxel Gridフィルタを使ったpcdファイルの間引き。複数Viewportの例
3. SACSegmentation: SACSegmentationを使った平面抽出とExtract Indicesによる点群の抽出
4. outlierRemoval: SACSegmentationとExtract Indicesで平面を除去した後のOutlier除去
5. outlierRemovalwVG: 2と4の組合せ。間引いたデータでoutlier除去
6. icpRegistration: フォルダ内にある複数のpcdファイルを対象として，4の方法で抽出した地上物体についてICPを使ってincremental registration
7. tranICPRegistration: 6でICPの前に直前の移動行列を反映させて一致しやすくさせる
8. regionGrowing: 4の結果について法線推定をした後でregionGrowingによるクラスタリング
9. GreedyProjTr: GreedyProjectionTriangulationでメッシュ化。あとPointNormalの使い方


※サンプルデータは[kitti2pcl](https://github.com/jaejunlee0538/kitti2pcl)でKITTI datasetから抽出したもの

voxelGrid
![](https://github.com/eiichiromomma/pcl_samples/blob/master/voxelGrid/voxelGrid-screenshot.png)

SACSegmentation
![](https://github.com/eiichiromomma/pcl_samples/blob/master/SACSegmentation/SACSegmentation-screenshot.png)

outlierRemoval
![](https://github.com/eiichiromomma/pcl_samples/blob/master/outlierRemoval/outlierRemoval-screenshot.png)

outlierRemovalwVG
![](https://github.com/eiichiromomma/pcl_samples/blob/master/outlierRemovalwVG/outlierRemovalwVG-screenshot.png)

icpRegistration
![](https://github.com/eiichiromomma/pcl_samples/blob/master/icpRegistration/icpRegistration-screenshot.png)

tranICPRegistration
![](https://github.com/eiichiromomma/pcl_samples/blob/master/tranICPRegistration/tranICPRegistration-screenshot.png)

regionGrowing
![](https://github.com/eiichiromomma/pcl_samples/blob/master/regionGrowing/regionGrowing-screenshot.png)

GreedyProjTr
![](https://github.com/eiichiromomma/pcl_samples/blob/master/GreedyProjTr/GreedyProjTr-screenshot.png)

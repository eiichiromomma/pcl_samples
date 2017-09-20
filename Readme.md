PCLのサンプル
---

1. pcdViewer: PCLVisualizerを使ったpcdファイルの表示
2. voxelGrid: Voxel Gridフィルタを使ったpcdファイルの間引き。複数Viewportの例
3. SACSegmentation: SACSegmentationを使った平面抽出とExtract Indicesによる点群の抽出
4. outlierRemoval: SACSegmentationとExtract Indicesで平面を除去した後のOutlier除去
5. outlierRemovalwVG: 2と4の組合せ。間引いたデータでoutlier除去
6. icpRegistration: フォルダ内にある複数のpcdファイルを対象として，4の方法で抽出した地上物体についてICPを使ってincremental registration



※サンプルデータは[kitti2pcl](https://github.com/jaejunlee0538/kitti2pcl)でKITTI datasetから抽出したもの
tranICPRegistration
---
SACSegmentationで路面を除去し，残った地上物体についてICPを使ってincremental registration。ICPの前に直前のICPによる移動行列をtransformPointCloudで反映させる。局所解が出にくい状況なら一定速度で移動し続ける限りincremental registrationが上手く行く。左がその結果で右は単純に追加した場合。

![](https://github.com/eiichiromomma/pcl_samples/blob/master/tranICPRegistration/tranICPRegistration-screenshot.png)

調子に乗ってDatasetを全部入れてみたら局所解でグチャグチャになった。

![](https://github.com/eiichiromomma/pcl_samples/blob/master/tranICPRegistration/tranICPRegistration-screenshot2.png)

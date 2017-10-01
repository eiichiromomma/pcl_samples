tranICPRegistrationQt
---
SACSegmentationで路面を除去し，残った地上物体についてICPを使ってincremental registration。ICPの前に直前のICPによる移動行列をtransformPointCloudで反映させる。局所解が出にくい状況なら一定速度で移動し続ける限りincremental registrationが上手く行く。左がその結果で右は単純に追加した場合。

の際にPCDファイルの置かれたフォルダを、単に起動した時はQFileDialog::getExistingDirectoryでダイヤログを開く。引数でフォルダが与えられた場合はそれを使う。

![](https://github.com/eiichiromomma/pcl_samples/blob/master/tranICPRegistration/tranICPRegistration-screenshot.png)

調子に乗ってDatasetを全部入れてみたら局所解でグチャグチャになった。

![](https://github.com/eiichiromomma/pcl_samples/blob/master/tranICPRegistration/tranICPRegistration-screenshot2.png)

### Windowsの場合、DLLのコピー等が必要

CMakeLists.txtで
```cmake
qt5_use_modules(pcdViewerQt Core Gui Widgets)
```
のように書かれた場合、C:\Qt\5.9.1\msvc2015_64\bin(アーキテクチャとバージョンは適宜置き換えて考える)からQt5Core.dll, Qt5Gui.dll, Qt5Widgets.dllをビルドした.exeと同じ場所に置き、更にplatformsフォルダを作成して、C:\Qt\5.9.1\msvc2015_64\plugins\platforms内のdllファイルをコピーする必要がある。(デバッグの場合はそれぞえdの付いたdllとpdbが必要)

```
│  pcdViewerQt.exe
│  Qt5Core.dll
│  Qt5Gui.dll
│  Qt5Widgets.dll
│
└─platforms
        qdirect2d.dll
        qminimal.dll
        qoffscreen.dll
        qwindows.dll
```
のような構造にしないと

```
This application failed to start because it could not find or load the Qt platform plugin "windows" in ""
```
というエラーが発生する。
pcdViewerQt
---

QtのQFileDialogを使ってファイル選択する例。.exeファイルをダブルクリック起動でpcd選択のダイヤログが開く。

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
# flir_camera_driver

# 環境
* ubuntu 22.04
* ros2 humble
* arm or amd アーキテクチャ

# 起動手順
## 1. Spinnaker SDKのインストール
[Flirの公式サイト](https://www.teledynevisionsolutions.com/products/spinnaker-sdk/?model=Spinnaker%20SDK&vertical=machine%20vision&segment=iis)より, Spinnaker Linuxをダウンロード.

ArmとAmdのアーキテクチャに注意し, 適したものを選択.

ダウンロードの後
```
tar -zcvf <xxxx.tar.gz>
cd spinnaker-x.x.x.x-xxxx/
./install_spinnaker.sh 
```
にてインストールを行う.

インストールの際に聞かれる選択肢は基本的にYesでOK.

途中ユーザー名を聞かれるので, ここの入力に注意.
```
This script will assist users in configuring their udev rules to allow
access to USB devices. The script will create a udev rule which will
add FLIR USB devices to a group called flirimaging. The user may also
choose to restart the udev daemon. All of this can be done manually as well.

Adding new members to usergroup flirimaging...
Current members of flirimaging group: usr
To add a new member please enter username (or hit Enter to continue):
$ 

```
自分がuser名として登録しているものを入力する必要がある.

> [!WARNING]
> 登録していないuser名にしたり設定せずに先に進んでしまうと, デバイスの認識が出来ず動かない.

既存のuser名がわからない場合は
```
cut -d: -f1 /etc/passwd
```
にて確認すればOK.

インストール完了の後は
```
newgrp flirimaging
```
でグループを反映させた後, インストールされたSpinViewアプリを立ち上がり, デバイスが認識されれば起動完了.

## 2. Ros2ドライバーの起動
このGitリポジトリがクローンされているという前提で, colcon build

いくつかの依存関係がないと思われるので適宜aptインストール

###  paramの変更
SpinViewの接続情報をみるとflirカメラのシリアルナンバーが分かるため,
```
src/spinnaker_camera_driver/launch/driver_node.launch.py
```
の中にあるシリアル設定を変更する.

起動は以下のコマンド
```
roslaunch spinnaker_sdk_camera_driver node_acquisition.launch 
```
Rviz2にて映像が出力されれば完成

## 3. RGB画像の出力
もし上記のドライバーからのデータでモノクロ画像が出力されていなる場合, データの変換をする必要がある.
### RGBデータをPublishしていることの確認
```
ros2 topic echo /flir_camera/image_raw
```
にて`encoding:bayer_gbrg8`となっていることを確認.
### image_procを用いる
インストール
```
sudo apt install ros-humble-image-pipeline
```
起動
```
ros2 run image_proc image_proc --ros-args -r __ns:=/flir_camera
```
ここで`--ros-args -r __ns:=/flir_camera`にてimage_rawをimage_procが取得できるようにトピック名の変換を行っている.
確認
```
ros2 topic list
```
出力に`flir_camera/image_color`が出ていればOK.

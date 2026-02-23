# Quadruped FK & IK

將本次作業package移到你的catkin_ws

```

$ cd ~/{your workspace}
$ catkin_make
$ source ~/.bashrc
$ source ~/{your workspace}/devel/setup.bash
```

## 檔案說明
**joint_states.txt :**  12軸關節角度（100 hz）。

**quadruped.pdf :** 定義機械狗關節座標系。

**joint_states_pub.py :**   讀取joint_states.txt，並以100 hz頻率發布到/joint_states

**FK.py :** 計算 dh_transform，並返回end_effector位置。

**quadruped_tf_broadcaster.py :**   利用FK將各軸座標系發布到 ROS tf。

**quadruped_env :** 模擬機械狗環境，給定初始角度，並根據 /joint_command 更新各軸角度。

**IK_jacobian.py :**    計算當前 Jacobian。

**squat.py :**  規劃蹲下起立的腳步 end_effector 軌跡，並發布 /joint_command。

## Forward Kinematic
```
$ rosrun quadruped quadruped_tf_broadcaster.py
$ rosrun quadrupped joint_states_pub.py
$ rviz
```
## Inverse Kinematic
```
$ rosrun quadruped quadruped_tf_broadcaster.py
$ rosrun quadrupped quadruped_env.py
$ rosrun quadrupped squat.py
$ rviz
```


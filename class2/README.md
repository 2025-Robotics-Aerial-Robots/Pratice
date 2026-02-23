# Practice2

- 只要編譯指定的package
```bash=
$ catkin_make -DCATKIN_WHITELIST_PACKAGES="practice2"
```

## 題目
1. 請實踐出一個使turtlesim走到你指定位置的功能，注意frame之間的轉換，turtlesim的座標定義在world frame ，但是他的twist定義在 body frame。
- 須更改 code : turtle_Pcontrol_W.cpp

![](https://i.imgur.com/uEb3wAr.png)


``` bash=
$ roslaunch practice2 turtle_single.launch
$ rosrun practice2 turtle_Pcontrol_W
```
----
2. 題目大致同上，不過本題的指定位置改為 body frame, 舉例：若目標為(2, 2)則烏龜要移動到以自己頭為X軸，移動(2, 2)，而非上一題的移動到 world frame (2, 2)。
- 須更改 code : turtle_Pcontrol_B.cpp

![45678 drawio (1)](https://user-images.githubusercontent.com/75787424/222947992-1b2d332e-7fd2-4286-aeb0-0d02756e11ea.png)

``` bash=
$ roslaunch practice2 turtle_single.launch
$ rosrun practice2 turtle_Pcontrol_B
```

----
3. (Bonus)請實現一個烏龜編隊的功能。共有三隻烏龜，一隻為 leader，其餘為 follower。
- 編隊隊形須定義再leader 的 body frame 底下，ex: leader頭轉，隊伍就要轉動
- leader turtle 由 keyboard control 來移動
- 須更改 code : turtle_formation.cpp

![](https://i.imgur.com/USt9MmZ.png)

```bash=
$ roslaunch practice2 turtle_team.launch
$ rosrun practice2 turtle_formation
$ rosrun practice2 turtle_keyboard
```

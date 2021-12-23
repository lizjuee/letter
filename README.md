# letter
使用Yolov3识别出不同字母的积木，并排列在指定位置


# 这个demo一共有三个功能包。分别是：
1. darknet_ros 机器视觉功能包
2. xarm_ros 机械臂运行功能包
3. grasp_demo 机械臂抓取代码的实现

# 运行该代码的流程
```
1. $cd /your_workspace

2. $git clone https://github.com/qq44642754a/letter.git

3. $catkin_make

4. $source devel/setup.bash

5. $roslaunch grasp_demo start.launch // 启动相关节点机械臂等

6. $roslaunch darknet_ros darknet_letter.launch // 启动机器视觉算法

7. $rosrun grasp_demo ggcnn_grasp //开始demo
```

# 注意：需要修改的地方：
1. 在CamtoReal 中需要把相机的topic改成自己相机对应的topic。例如：/cam_1/aligned_depth_to_color/image_raw -> /camera/aligned_depth_to_color/image_raw 
2. 需要在objtobaseserver.py把tool_h_cam改成你自己的相对应的手眼的参数。
3. 修改在darknet_ros 功能年包下的 launch/darknet_letter.launch , config/ros.yaml 的文件。把相机的topic改成自己相机对应的topic。
4. 在letter_grasp 中，需要在启动前修改初始点和机械臂放置的点。

# Demo 说明
这个demo是一个输入单词，然后通过机器视觉和机械臂的作用下，对带有字母的木块进行拼接。

## 代码的整体流程：
1. demo开始机械臂会运动到预设的初始的位置（本demo是在木块的上方）对应为代码中的函数 goSP()
2. 接着系统会询问你需要拼接的单词，按回车键确认输入。 例如输入hello
3. 将输入的单词按照其组成的字母分开。例如hello，则为h，e，l， l和o 。
4. 分别的去抓取每个字母。抓取的过程细分为识别物体位置，位置的坐标变换
	4.1 首先将识别到的字母传入函数getPose(string Object)中。例如getPose('h')则可获得字母h对应的木块的像素坐标
	4.2 接着需要把这个像素坐标转化为在机械臂基坐标系下的坐标。函数getObjPose()的调用就能获得例如字母“h”的坐标。
5. 机械臂运动到相应位置。 调用函数goObj()，机械臂就会运动到第四步识别出物体坐标
6. 抓取物体。调用函数graspObj()
7. 移动物体。调用函数moveObj()
8. 通过上述的这些函数即可实现这个demo。最后在initMove中对上述的过程进行实现。

## 同时这个代码用到了三个功能接口。
1. 机械臂的坐标变化 （scripts/objtobaseserver.py）
2. 像素坐标映射到三维坐标。即通过物体识别得到的像素坐标来获取物体距离相机的实际坐标xyz(CamtoReal.cpp)。
3. 获得制定物体的像素坐标(通过机器视觉识别到物体，并输出像素坐标)(yolo_obj.cpp)

### 这三个功能接口通过rosservice来调用。分别是:
```
rosservice call /cam_to_real "pixel_x: 0.0
pixel_y: 0.0"
```
输入像素坐标即可获得 xyz
```
rosservice call /objtobaselink  "{marker_x: 0.0, marker_y: 0.0, marker_z: 0.0, robot_x: 0.0, robot_y: 0.0, robot_z: 0.0,
  robot_roll: 0.0, robot_pitch: 0.0, robot_yaw: 0.0}"
 ```
坐标转化接口需要输入物体的xyz，以及机械臂当前状态的xyz和rpy。
```
rosservice call /get_object "obj: 'e'" 
```
需要输入识别的物体，即可获得物体的像素位置

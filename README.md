# letter
identify workpiece with letter use Yolov3, then sort it in Specific location.


# this demo has three package：
1. darknet_ros (robot vision function package)
2. xarm_ros (xarm motion package)
3. grasp_demo (code of xarm grasp)

# The process of runs the code
```
1. $cd /your_workspace

2. $git clone https://github.com/qq44642754a/letter.git

3. $catkin_make

4. $source devel/setup.bash

5. $roslaunch grasp_demo start.launch // launch corresponding nodes include xarm

6. $roslaunch darknet_ros darknet_letter.launch // launch robot vision algorithm

7. $rosrun grasp_demo ggcnn_grasp //start demo
```

# ps：These places need to modified：
1. modify the camera topic to your camera in "CamtoReal", for example：/cam_1/aligned_depth_to_color/image_raw -> /camera/aligned_depth_to_color/image_raw 
2. modify "tool_h_cam" in "objtobaseserver.py"to your corresponding hand eye parameters.
3. modify" launch/darknet_letter.launch" in "darknet_ros"package , modify camera topic in "config/ros.yaml" to your corresponding topic。
4. modify the initial position and target position you want xarm to go in"letter_grasp".

# Demo discription
this is a demo that input a word，then use machine vision and xarm to grasp the workpiece with letter to a specific location.

## 代码的整体流程：
1. when demo start, xarm will move to the initial position（it is above the workpiece in this demo）,corresponding function in program is goSP().
2. then the system will ask What word do you want to splice，input your word and press "enter" button. for example input"hello".
3. the program will split the word into letter,for example "hello" to "h，e，l， l and o".
4. locate the letters. the process will be divide into two parts:find the location of the workpiece，Coordinate transformation of position.
	4.1 the program first transform the recognized letter to function"getPose(string Object)". for example"getPose('h')" will get the position of letter"h" in pixel.
	4.2 then transform the pixel location to xarm Base coordinates location. function "getObjPose()" is used to do the coordinate transformation work.
5. xarm move to a corresponding position. call function"goObj()", xarm will move to the position above the workpiece to get the coordinate.
6. grasp workpiece. call function "graspObj()"
7. move workpiece. call function "moveObj()"
8. use the functions above can accomplish the demo. Finally, the above process is implemented in initmove。

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

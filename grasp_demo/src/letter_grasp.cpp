/*这个demo是一个输入单词，然后通过机器视觉和机械臂的作用下，对带有字母的木块进行拼接。

代码的整体流程：
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


 Author：Simple
 v : zhengzhitao999

*/

#include "letter_grasp.h"

grasp::grasp() : armgroup("xarm6"), move_finish(false), joint_group_positions(6), find_enable(false), yolo_enable(false)
{
    try
    {
        //获取终端link的名称
        end_effector_link = armgroup.getEndEffectorLink();

        //设置目标位置所使用的参考坐标系
        reference_frame = "link_base";
        armgroup.setPoseReferenceFrame(reference_frame);

        //当运动规划失败后，允许重新规划
        armgroup.allowReplanning(false);

        //设置位置(单位：米)和姿态（单位：弧度）的允许误差
        armgroup.setGoalPositionTolerance(0.001);
        armgroup.setGoalOrientationTolerance(0.01);

        //设置允许的最大速度和加速度
        armgroup.setMaxAccelerationScalingFactor(0.1);
        armgroup.setMaxVelocityScalingFactor(0.1);

        //设置初始点
        joint_group_positions[0] = 0.0 / 180.0 * M_PI;
        joint_group_positions[1] = -21.7 / 180.0 * M_PI;
        joint_group_positions[2] = -64.7 / 180.0 * M_PI;
        joint_group_positions[3] = 0.0 / 180.0 * M_PI;
        joint_group_positions[4] = 86.4 / 180.0 * M_PI;
        joint_group_positions[5] = 0.0 / 180.0 * M_PI;
    }
    catch (const std::exception &e)
    {
        ROS_WARN_STREAM("run error: " << e.what());
    }
}

grasp::~grasp()
{
    ROS_INFO("Delete the class");
}

void grasp::gripper_open()
{
    actionlib::SimpleActionClient<xarm_gripper::MoveAction> ac("xarm/gripper_move", true);
    ac.waitForServer();
    xarm_gripper::MoveGoal open;
    open.target_pulse = 850;
    open.pulse_speed = 4000;
    ac.sendGoal(open);
}

void grasp::gripper_close()
{
    actionlib::SimpleActionClient<xarm_gripper::MoveAction> ac("xarm/gripper_move", true);
    ac.waitForServer();
    xarm_gripper::MoveGoal close;
    close.target_pulse = 0;
    close.pulse_speed = 2000;
    ac.sendGoal(close);
}

void grasp::goSP()
{
    armgroup.setJointValueTarget(joint_group_positions);
    armgroup.move();
}

void grasp::getPose(string object)
{
    ros::service::waitForService("get_object"); // 调用物体检测接口，返回物体的像素坐标
    ros::ServiceClient object_client = nh_.serviceClient<grasp_demo::get_obj>("get_object");
    grasp_demo::get_obj srv1;
    srv1.request.obj = object;
    object_client.call(srv1);
    pixel_x = srv1.response.x; // 返回的物体的像素坐标 x
    pixel_y = srv1.response.y; // 返回的物体的像素坐标 y

    ros::service::waitForService("cam_to_real");
    ros::ServiceClient pose_client = nh_.serviceClient<grasp_demo::cam2real>("cam_to_real");//调用像素坐标映射到三维坐标接口，传入像素坐标，返回物体距离机械臂的实际坐标xyz
    grasp_demo::cam2real srv;
    srv.request.pixel_x = pixel_x; // 输入像素坐标x
    srv.request.pixel_y = pixel_y; // 输入像素坐标y

    Obj_pose.pose.position.x = 0;
    int attempt = 0;
    while (Obj_pose.pose.position.x == 0 && attempt < 5)
    {
        pose_client.call(srv);
        Obj_pose.pose.position.x = srv.response.obj_x; // 返回物体距离机械臂的实际坐标xyz
        Obj_pose.pose.position.y = srv.response.obj_y; 
        Obj_pose.pose.position.z = srv.response.obj_z;
        attempt += 1;
        cout << Obj_pose.pose.position.z << endl;
    }
    
}

void grasp::getObjPose()
{
    vector<double> rpy_;
    geometry_msgs::PoseStamped current_pose_;
    rpy_ = armgroup.getCurrentRPY();
    current_pose_ = armgroup.getCurrentPose();
    ros::service::waitForService("objtobaselink");
    ros::ServiceClient client = nh_.serviceClient<grasp_demo::objtobaselink>("objtobaselink"); // 调用机械臂的坐标变化
    grasp_demo::objtobaselink srv;

    /* 坐标转化接口需要输入物体的xyz，以及机械臂当前状态的xyz和rpy。*/
    srv.request.marker_x = Obj_pose.pose.position.x;
    srv.request.marker_y = Obj_pose.pose.position.y;
    srv.request.marker_z = Obj_pose.pose.position.z;
    srv.request.robot_x = current_pose_.pose.position.x;
    srv.request.robot_y = current_pose_.pose.position.y;
    srv.request.robot_z = current_pose_.pose.position.z;
    srv.request.robot_roll = rpy_[0];
    srv.request.robot_pitch = rpy_[1];
    srv.request.robot_yaw = rpy_[2];

    /*得到物体在机械臂基坐标系下的坐标 */
    if (client.call(srv))
    {
        target_pose.position.x = srv.response.obj_x;
        target_pose.position.y = srv.response.obj_y;
        target_pose.position.z = srv.response.obj_z + 0.05;

        quaternion = Eigen::AngleAxisd(rpy_[2], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(rpy_[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(rpy_[0], Eigen::Vector3d::UnitX());

        target_pose.orientation.x = quaternion.x();
        target_pose.orientation.y = quaternion.y();
        target_pose.orientation.z = quaternion.z();
        target_pose.orientation.w = quaternion.w();
        cout << target_pose << endl;
    }
    else
    {
        ROS_ERROR("Failed to call service ObjtoBaseLink");
    }
}

void grasp::goObj()
{
    armgroup.setPoseTarget(target_pose);
    armgroup.move();
}

void grasp::graspObj()
{
    armgroup.setMaxAccelerationScalingFactor(0.05);
    armgroup.setMaxVelocityScalingFactor(0.05);
    target_pose.position.z -= 0.07;
    armgroup.setPoseTarget(target_pose);
    armgroup.move();
    gripper_close();
    sleep(3);
}

void grasp::moveObj(int i)
{
    //lift object
    target_pose.position.z += 0.15;
    armgroup.setPoseTarget(target_pose);
    armgroup.move();

    // arrange in word order
    target_pose.position.x = 0.3392;
    target_pose.position.y = -0.2516 - 0.1*i;
    armgroup.setPoseTarget(target_pose);
    armgroup.move();

    // put down object
    target_pose.position.z -= 0.15;
    armgroup.setPoseTarget(target_pose);
    armgroup.move();

    gripper_open();

    // raise arm
    target_pose.position.z += 0.15;
    armgroup.setPoseTarget(target_pose);
    armgroup.move();

}

void grasp::goHome()
{
    armgroup.setJointValueTarget(joint_group_positions);
    armgroup.move();
    sleep(0.5);
    armgroup.setNamedTarget("home");
    armgroup.move();
}

void grasp::initMove()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    gripper_open(); // 打开夹爪
    sleep(1);
    goSP(); // 运动到预设的初始的位置
    string str;
    char temp;
    ROS_INFO_STREAM("please input the word?  press Enter to continue"); // 输入单词
    while ((temp = cin.get()) != '\n')
    {
        str += temp;
    }
    const int LEN = str.length();
    string as [100];
    for (int i =0; i < LEN ; i++)
    {
        as[i] = str[i]; // 将输入的单词按照其组成的字母分开
        getPose(as[i]); //得到物体的离相机的位置
        getObjPose(); //得到物体在机械臂基坐标系下的位置
        goObj(); //去到物体上面
        graspObj(); //抓取物体
        moveObj(i); //移动物体
        Obj_pose.pose.position.x = 0;
        //gripper_open();
        sleep(1);
        goSP();
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grasp_obj");
    grasp _grasp;
    while (ros::ok())
    {
        _grasp.initMove();
        ros::spinOnce();
        if (_grasp.move_finish)
        {
            ros::shutdown();
        }
    }
}

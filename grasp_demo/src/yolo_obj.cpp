/*该接口是获得指定物体的像素坐标(通过机器视觉识别到物体，并输出像素坐标)


 Author：Simple
 v : zhengzhitao999

*/

#include "yolo_obj.h"

grasp::grasp() : move_finish(false)
{
    try
    {
        get_object = nh_.advertiseService("get_object", &grasp::getObjectCallback, this);
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

void grasp::yoloCallback(const darknet_ros_msgs::BoundingBoxes &yolo_tmp)
{
   auto num = yolo_tmp.bounding_boxes[0].id;
    for (int i = 0; i < num; i++)
    {
        if (strcmp(yolo_tmp.bounding_boxes[i].Class.c_str(), Object_class.c_str()) == 0)
        {
            pixel_x = (yolo_tmp.bounding_boxes[i].xmin + yolo_tmp.bounding_boxes[i].xmax) / 2;
            pixel_y = (yolo_tmp.bounding_boxes[i].ymin + yolo_tmp.bounding_boxes[i].ymax) / 2;
        }
    }
}

void grasp::initMove()
{
    ros::AsyncSpinner spinner(1);
    spinner.start();
    object_sub = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &grasp::yoloCallback, this);
    sleep(3);
    object_sub.shutdown();
}

bool grasp::getObjectCallback(grasp_demo::get_obj::Request &req,
                              grasp_demo::get_obj::Response &res)
{
    Object_class = req.obj;
    initMove();
    res.x = pixel_x;
    res.y = pixel_y;

    if (res.x != 0)
    {
        res.result = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_obj");
    while (ros::ok())
    {
        grasp _grasp;
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::waitForShutdown();
    }
}

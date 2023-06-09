#include <ros/ros.h>
#include <stdio.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "extrinsics_calc_node");
    ros::NodeHandle nh("~");

    // 从lidar到event的变换矩阵
    Eigen::Matrix4d T_event_lidar, T_camera_lidar, T_camera_event;
    Eigen::Matrix4d T_event_camera;
    T_camera_event << 0.9998732356434525, 0.01166113698213495, -0.01084114976267556, -0.0007543180009142757,
        -0.01183095928451621, 0.9998062047518974, -0.01573471772912168, -0.04067615384902421, 0.01065556410055307,
        0.01586098432919285, 0.9998174273985267, -0.01466127320771003, 0, 0, 0, 1;

    T_event_camera = T_camera_event.inverse();

    T_camera_lidar << 0.0119197, -0.999929, 0.0000523, 0.0853154, -0.00648951, -0.00012969, -0.999979, -0.0684439,
        0.999908, 0.0119191, -0.0064906, -0.0958121, 0, 0, 0, 1;

    T_event_lidar = T_event_camera * T_camera_lidar;

    std::cout << "T_event_lidar = \n" << T_event_lidar << std::endl;

    return 0;
}
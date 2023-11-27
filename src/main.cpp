/**
 * @file main.cpp
 * @author zzzing (1226196281@qq.com)
 * @brief 串口库主函数，包含一些调用示例
 * @version 2.1
 * @date 2023-10-12
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "main.hpp"

using namespace std;
using namespace Eigen;
std::string com_name;
Uart_Thread uart;

/*任务1*/
void thread1(ros::NodeHandle &n);
/*任务2*/
void thread2(ros::NodeHandle &n);
/*回调函数*/
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    int main(int argc, char **argv)
{
    ros::init(argc, argv, "uart_pub");
    ros::NodeHandle n;
    n.param<std::string>("com_name", com_name, "/dev/ttyUSB0");

    // uart初始化
    uart.InitSerialPort(com_name);

    // 开启读写串口线程
    uart.Enable_Thread_Read_Uart();
    
    thread th1(thread1,std::ref(n));
    //thread th2(thread2,std::ref(n));

    ros::spin();
    return 0;
}

/*任务1*/
void thread1(ros::NodeHandle & n)
{
    ros::Subscriber sub_odom = n.subscribe<nav_msgs::Odometry>("/Odometry", 1000, odomCallback);
    ros::Rate loop_rate(100); // 任务执行频率为1Hz
    while (ros::ok())
    {
        ros::spinOnce();   // 处理回调函数
        loop_rate.sleep(); // 控制循环频率
    }
}

/*任务2
void thread2(ros::NodeHandle &n)
{
    
    while (ros::ok())
    {
        ros::Subscriber sub_pcl = n.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/pcl", 1, pclCallback);
        ros::Rate loop_rate(1); // 任务执行频率为2Hz
        X++;
        // to do ...

        *完成任务1后，发送一次串口，记得赋值时使用强制类型转换保证数据类型一致*
        uart.Mission_Send(Uart_Thread_Space::Mission2_Assignment, &uart, (uint16_t)X);
    }
}
*/

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float z = msg->pose.pose.position.z;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    double roll, pitch, yaw;                      
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 四元数转欧拉角
    roll=roll * 360 / (2 * M_PI);
    pitch=pitch * 360 / (2 * M_PI);
    yaw=yaw * 360 / (2 * M_PI);

    cout << "x: " << msg->pose.pose.position.x << " "
         << "y: " << msg->pose.pose.position.y << " "
         << "yaw: " << yaw << endl;

    float x_rounded = std::round(x * 1000.0) / 1000.0;
    float y_rounded = std::round(y * 1000.0)/1000.0;
    float z_rounded = std::round(z * 1000.0)/1000.0;
    float roll_rounded = std::round(roll * 1000.0) / 1000.0;
    float pitch_rounded = std::round(pitch * 1000.0) / 1000.0;
    float yaw_rounded = std::round(yaw * 1000.0) / 1000.0;

    /*完成任务1后，发送一次串口，记得赋值时使用强制类型转换保证数据类型一致*/
    uart.Mission_Send(Uart_Thread_Space::Mission1_Assignment, &uart, x_rounded, y_rounded, yaw_rounded);
}
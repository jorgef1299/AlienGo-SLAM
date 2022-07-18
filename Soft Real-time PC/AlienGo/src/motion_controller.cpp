#include "onboard_pc_locomotion_control.h"
#include <geometry_msgs/Twist.h>
#include <csignal>

uint8_t operating_mode = 0;
double vlin = 0.0, omega = 0.0;
bool stop = false;
ros::Publisher pub_imu;

void cbVelocity(const geometry_msgs::TwistConstPtr &msg) {
    vlin = msg->linear.x;
    omega = msg->angular.x;
    if(msg->angular.z >= 0 && msg->angular.z <= 1)
        operating_mode = msg->angular.z;
    if(operating_mode == 0) {
        ROS_INFO("Fast running mode: Vlin = %.2f    Omega = %.2f", vlin, omega);
    }
    else if(operating_mode == 1) {
        ROS_INFO("Stair climbing mode: Vlin = %.2f    Omega = %.2f", vlin, omega);
    }
}

void signal_handler(int signal_num)
{
    ROS_INFO("Parei o robot");
    vlin = 0;
    omega = 0;
    stop = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_controller");
    ros::NodeHandle n_public;

    signal(SIGINT, signal_handler);

    ros::Subscriber sub_velocity = n_public.subscribe("/cmd_vel", 10, cbVelocity);

    Custom custom(HIGHLEVEL);
    custom.pub_imu = n_public.advertise<sensor_msgs::Imu>("/Aliengo/imu", 10);

    InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();
    return 0;

}

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

void Custom::RobotControl()
{
    if(motiontime <6000)
        motiontime += 2;
    udp.GetRecv(state);

    sensor_msgs::Imu msg_imu;
    msg_imu.header.stamp=ros::Time::now();
    msg_imu.header.frame_id="livox_frame";
    msg_imu.orientation.w = 0;
    msg_imu.orientation.x = 0;
    msg_imu.orientation.y = 0;
    msg_imu.orientation.z = 0;
    msg_imu.angular_velocity.x = state.imu.gyroscope[0];
    msg_imu.angular_velocity.y = state.imu.gyroscope[1];
    msg_imu.angular_velocity.z = state.imu.gyroscope[2];
    msg_imu.linear_acceleration.x = state.imu.accelerometer[0];
    msg_imu.linear_acceleration.y = state.imu.accelerometer[1];
    msg_imu.linear_acceleration.z = state.imu.accelerometer[2];
    pub_imu.publish(msg_imu);

    // make sure of 2m space in front of the robot head before starting this test
    if (motiontime == 2)
    {
        std::cout<<"begin sending commands."<<std::endl;
    }
    else if (motiontime>10 && motiontime <100)
    {
        cmd = {0};
        cmd.levelFlag = 0xf0;
        udp.SetSend(cmd);
    }
    else if (motiontime == 100)
    {
        std::cout<<"Aliengo sport mode trigger sent !"<<std::endl;

    }
    else if (motiontime >= 4000 && motiontime <4500)
    {
        cmd = {0};
        cmd.mode = 1; // to force stand status
        udp.SetSend(cmd);
    }
    else if (motiontime == 4500)
    {
        std::cout<<"Finshed ! Robot in stand. "<<std::endl;
    }
    else if (motiontime > 4500 && motiontime < 4600)
    {
        cmd = {0};
        cmd.mode = 0;
        cmd.gaitType = 0;
        udp.SetSend(cmd);
    }
    else {
        if(stop) cmd.mode = 0;
        else {
            if(std::abs(vlin) < 0.00005 && std::abs(omega) < 0.00005) {
                if(operating_mode == 1) cmd.mode = 1;
                else {
                    cmd = {0};
                    cmd.mode = 0;
                    cmd.gaitType = 0;
                    udp.SetSend(cmd);
                }
            }
            else {
                ROS_INFO("Setting velocity");
                cmd.mode = 2;    // mode 2: following target velocity in body frame
                if(operating_mode == 1) cmd.gaitType = 2; // stair climbing gait
                else cmd.gaitType = 0;
                cmd.velocity[0] = vlin;
                cmd.velocity[1] = 0;
                cmd.yawSpeed = omega;
            }
        }
        udp.SetSend(cmd);
    }
}


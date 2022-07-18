#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <mutex>
#include <Eigen/Eigen>
#include <math.h>
#include <iostream>
#include <csignal>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "atomic"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Int8.h>
#include "sensor_msgs/PointCloud2.h"

typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudXYZI;
PointCloudXYZI::Ptr map_cloud(new PointCloudXYZI());
PointCloudXYZI::Ptr bodyFiltered(new PointCloudXYZI());

std::mutex mtx_pose;
Eigen::Vector4d robot_pose, goal_pose; // x,y, z, theta

ros::Publisher pub_velocity;

tf2_ros::Buffer tfBuffer;
uint8_t robot_operating_mode = 0;

std::atomic_bool stop_program(false);
std::atomic_bool robot_lost(false);

bool run_go_to_xy = false;

double normalize_angle(double angle) {
    double normalized_angle;
    normalized_angle = angle;
    while(normalized_angle > M_PI) {
        normalized_angle -= 2 * M_PI;
    }
    while(normalized_angle <= -M_PI) {
        normalized_angle += 2 * M_PI;
    }
    return normalized_angle;
}

double calculate_dist(double x, double y) {
    double dist;
    dist = sqrt(x*x + y*y);
    return dist;
}

void cbCurrentRobotPose(const nav_msgs::OdometryConstPtr &msg) {
    // Convert quaternion orientation to RPY
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // Set Robot pose
    mtx_pose.lock();
    robot_pose[0] = msg->pose.pose.position.x;
    robot_pose[1] = msg->pose.pose.position.y;
    robot_pose[2] = msg->pose.pose.position.z;
    robot_pose[3] = yaw;
    mtx_pose.unlock();
}

void cbNewGoal(const geometry_msgs::PoseStampedConstPtr &msg) {
    // Get desired final yaw
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[3] = yaw;

    // Estimate the z component
    pcl::CropBox<pcl::PointXYZINormal> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(msg->pose.position.x-0.05, msg->pose.position.y - 0.2, robot_pose[2]-3, 1.0));
    boxFilter.setMax(Eigen::Vector4f(msg->pose.position.x+0.2, msg->pose.position.y + 0.2, robot_pose[2]+3, 1.0));
    boxFilter.setInputCloud(map_cloud);
    boxFilter.filter(*bodyFiltered);

    srand(time(NULL));
    double average_z = 0;
    int i;
    for(i = 0; i < 10 && i < bodyFiltered->points.size(); i++) {
        average_z += bodyFiltered->points[rand() % bodyFiltered->points.size() + 1].z;
    }
    goal_pose[2] = average_z / i;

    // Transform the goal pose to the robot's frame
    geometry_msgs::PointStamped src_pt, transformed_pt;
    src_pt.header.stamp = ros::Time::now();
    src_pt.header.frame_id = "camera_init";
    src_pt.point.x = goal_pose[0];
    src_pt.point.y = goal_pose[1];
    src_pt.point.z = goal_pose[2];
    tfBuffer.transform(src_pt, transformed_pt, "body", ros::Duration(1.0));

    // Calculate the steepness between the robot's current pose and the goal pose, in the robot's sagittal plane
    float steepness = std::abs(transformed_pt.point.z+0.7) / std::abs(transformed_pt.point.x);
    ROS_INFO("Declive = %f", steepness);
    if(std::abs(steepness) > 0.23) {
        robot_operating_mode = 1; // Stair climbing
    }
    else
        robot_operating_mode = 0; // Fast running
    run_go_to_xy = true;
}

void cbRobotState(const std_msgs::Int8& msg)
{
    if(msg.data == 0) robot_lost = true;
    else robot_lost = false;
}

class TGoToXYTheta {
public:
    TGoToXYTheta();
    void goToXYTheta(double x_f, double y_f, double theta_f);
    void update_ROS_parameters();
    float param_MAX_ETF;
    float param_TOL_FINDIST;
    float param_DIST_DA;
    float param_HIST_ETF;
    float param_DIST_NEW_POSE;
    float param_THETA_NEW_POSE;
    float param_THETA_DA;
    float param_TOL_FINTHETA;
    float param_VEL_ANG_NOM;
    float param_VEL_LIN_NOM;
    float param_GAIN_FWD;
    float param_VEL_LIN_DA;
    float param_GAIN_DA;
    float param_VEL_ANG_DA;
    double vlin, omega;
private:
    enum State {
        Rotation,
        Go_Forward,
        De_Accel,
        Final_Rot,
        DeAccel_Final_Rot,
        Stop
    } state;
};

TGoToXYTheta::TGoToXYTheta()
{
    state = Stop;
}

void TGoToXYTheta::goToXYTheta(double x_f, double y_f, double theta_f)
{
    double ang_target, error_ang, error_dist, error_final_rot;
    int8_t rotateTo, rotateToFinal;

    // Calculate actual errors
    ang_target = atan2(y_f-robot_pose[1], x_f-robot_pose[0]);
    error_ang = normalize_angle(ang_target-robot_pose[3]);
    error_dist = calculate_dist(x_f-robot_pose[0],y_f-robot_pose[1]);
    error_final_rot = normalize_angle(theta_f-robot_pose[3]);

    // Find fastest rotation
    if(error_ang > 0) {
        rotateTo = 1;
    }
    else{
        rotateTo = -1;
    }
    if(error_final_rot > 0) {
        rotateToFinal = 1;
    }
    else{
        rotateToFinal = -1;
    }

    // Transitions
    if(state == Rotation) {
        if(std::abs(error_ang) < param_MAX_ETF)
            state = Go_Forward;
        else if(error_dist < param_TOL_FINDIST)
            state = Final_Rot;
    }
    else if(state == Go_Forward) {
        if(error_dist < param_TOL_FINDIST)
            state = Final_Rot;
        else if(error_dist < param_DIST_DA)
            state = De_Accel;
        else if(std::abs(error_ang) > param_MAX_ETF + param_HIST_ETF)
            state = Rotation;
    }
    else if(state == De_Accel) {
        if(error_dist < param_TOL_FINDIST)
            state = Final_Rot;
        else if(error_dist > param_DIST_NEW_POSE)
            state = Rotation;
    }
    else if(state == Final_Rot) {
        if(std::abs(error_final_rot) < param_THETA_DA)
            state = DeAccel_Final_Rot;
        else if(error_dist > param_DIST_NEW_POSE)
            state = Rotation;
    }
    else if(state == DeAccel_Final_Rot) {
        if(std::abs(error_final_rot) < param_TOL_FINTHETA || robot_operating_mode == 1) {
            state = Stop;
            run_go_to_xy = false;
        }
        else if(error_dist > param_DIST_NEW_POSE || std::abs(error_final_rot) > param_THETA_NEW_POSE)
            state = Rotation;
    }
    else if(state == Stop) {
        if(error_dist > param_DIST_NEW_POSE || std::abs(error_final_rot) > param_THETA_NEW_POSE)
            state = Rotation;
    }

    // Outputs
    if(state == Rotation) {
        vlin = 0;
        omega = rotateTo * param_VEL_ANG_NOM;
    }
    else if(state == Go_Forward) {
        vlin = param_VEL_LIN_NOM;
        omega = param_GAIN_FWD * error_ang;
    }
    else if(state == De_Accel) {
        vlin = param_VEL_LIN_DA;
        omega = param_GAIN_DA * error_ang;
    }
    else if(state == Final_Rot) {
        vlin = 0;
        omega = rotateToFinal * param_VEL_ANG_NOM;
    }
    else if(state == DeAccel_Final_Rot) {
        vlin = 0;
        omega = rotateToFinal * param_VEL_ANG_DA;
    }
    else if(state == Stop) {
        vlin = 0;
        omega = 0;
    }

    if(robot_operating_mode == 1) {
        vlin /= 3;
    }
    geometry_msgs::Twist msg;
    msg.linear.x = vlin;
    msg.angular.x = omega;
    msg.angular.z = robot_operating_mode;
    pub_velocity.publish(msg);
}

void TGoToXYTheta::update_ROS_parameters()
{
    ros::param::get("GoToXYTheta/Max_ETF", param_MAX_ETF);
    ros::param::get("GoToXYTheta/TOL_FINDIST", param_TOL_FINDIST);
    ros::param::get("GoToXYTheta/DIST_DA", param_DIST_DA);
    ros::param::get("GoToXYTheta/HIST_ETF", param_HIST_ETF);
    ros::param::get("GoToXYTheta/DIST_NEWPOSE", param_DIST_NEW_POSE);
    ros::param::get("GoToXYTheta/THETA_DA", param_THETA_DA);
    ros::param::get("GoToXYTheta/TOL_FINTHETA", param_TOL_FINTHETA);
    ros::param::get("GoToXYTheta/THETA_NEWPOSE", param_THETA_NEW_POSE);
    ros::param::get("GoToXYTheta/VEL_ANG_NOM", param_VEL_ANG_NOM);
    ros::param::get("GoToXYTheta/VEL_LIN_NOM", param_VEL_LIN_NOM);
    ros::param::get("GoToXYTheta/GAIN_FWD", param_GAIN_FWD);
    ros::param::get("GoToXYTheta/VEL_LIN_DA", param_VEL_LIN_DA);
    ros::param::get("GoToXYTheta/GAIN_DA", param_GAIN_DA);
    ros::param::get("GoToXYTheta/VEL_ANG_DA", param_VEL_ANG_DA);
    if(robot_operating_mode == 1) {
        param_TOL_FINTHETA *= 2;
    }
}

TGoToXYTheta MotionControllerGoTOXYTheta;

void initialize_map(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::VoxelGrid<pcl::PointXYZINormal> downSizeFilterSurf;
    downSizeFilterSurf.setLeafSize(0.2, 0.2, 0.2);
    pcl::fromROSMsg(*msg, *map_cloud);
    downSizeFilterSurf.setInputCloud(map_cloud);
    downSizeFilterSurf.filter(*map_cloud);
}

void signal_handler(int signal_num)
{
    MotionControllerGoTOXYTheta.vlin = 0;
    MotionControllerGoTOXYTheta.omega = 0;
    stop_program = true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh;

    nh.param<float>("GoToXYTheta/Max_ETF", MotionControllerGoTOXYTheta.param_MAX_ETF, 0.1745);
    nh.param<float>("GoToXYTheta/TOL_FINDIST", MotionControllerGoTOXYTheta.param_TOL_FINDIST, 0.2);
    nh.param<float>("GoToXYTheta/DIST_DA", MotionControllerGoTOXYTheta.param_DIST_DA, 0.5);
    nh.param<float>("GoToXYTheta/HIST_ETF", MotionControllerGoTOXYTheta.param_HIST_ETF, 0.1745);
    nh.param<float>("GoToXYTheta/DIST_NEWPOSE", MotionControllerGoTOXYTheta.param_DIST_NEW_POSE, 0.5);
    nh.param<float>("GoToXYTheta/THETA_DA", MotionControllerGoTOXYTheta.param_THETA_DA, 0.43625);
    nh.param<float>("GoToXYTheta/TOL_FINTHETA", MotionControllerGoTOXYTheta.param_TOL_FINTHETA, 0.08725);
    nh.param<float>("GoToXYTheta/THETA_NEWPOSE", MotionControllerGoTOXYTheta.param_THETA_NEW_POSE, 0.45);
    nh.param<float>("GoToXYTheta/VEL_ANG_NOM", MotionControllerGoTOXYTheta.param_VEL_ANG_NOM, 0.25);
    nh.param<float>("GoToXYTheta/VEL_LIN_NOM", MotionControllerGoTOXYTheta.param_VEL_LIN_NOM, 0.2);
    nh.param<float>("GoToXYTheta/GAIN_FWD", MotionControllerGoTOXYTheta.param_GAIN_FWD, 1);
    nh.param<float>("GoToXYTheta/VEL_LIN_DA", MotionControllerGoTOXYTheta.param_VEL_LIN_DA, 0.1);
    nh.param<float>("GoToXYTheta/GAIN_DA", MotionControllerGoTOXYTheta.param_GAIN_DA, 0.5);
    nh.param<float>("GoToXYTheta/VEL_ANG_DA", MotionControllerGoTOXYTheta.param_VEL_ANG_DA, 0.1);

    tf2_ros::TransformListener tfListener(tfBuffer);

    sensor_msgs::PointCloud2ConstPtr map_msg;
    map_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map");
    initialize_map(map_msg);

    ros::Subscriber sub_current_pose = nh.subscribe("/Odometry", 10, cbCurrentRobotPose);
    ros::Subscriber sub_new_goal = nh.subscribe("/move_base_simple/goal", 10, cbNewGoal);
    ros::Subscriber sub_robot_state = nh.subscribe("/robot_state", 10, cbRobotState);
    pub_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    signal(SIGINT, signal_handler);

    ros::Rate rate(20);
    while(true) {
        ros::spinOnce();
        if(run_go_to_xy && !robot_lost)
            MotionControllerGoTOXYTheta.goToXYTheta(goal_pose[0], goal_pose[1], goal_pose[3]);
        rate.sleep();
        if(stop_program) break;
    }

    return 0;
}

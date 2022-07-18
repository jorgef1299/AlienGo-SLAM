#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <thread>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include "common_lib.h"
#include <queue>
#include <csignal>
#include <atomic>
#include <chrono>
#include <mutex>
#include "fast_lio/OptimizePose.h"
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"

#define NUM_SCANS_TO_CONCATENATE 10
#define MAP_FILTER_SIZE 0.1f
#define GLOBAL_LOCALIZ_FREQ 0.5f

std::mutex mtx_lidar;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
bool needed_initial_estimate = false;
double initial_pose_time;

PointCloudXYZI::Ptr local_scan;
PointCloudXYZI::Ptr map_pcl_cloud;
PointCloudXYZI last_lidar_scan_in_local_map;

ros::Subscriber sub_lidar_scan;
ros::Publisher pub_concatenated_scan;
ros::Publisher pub_registered_scan;
double last_lidar_scan_time;

shared_ptr<Preprocess> p_pre(new Preprocess());

std::atomic<bool> exit_program(false);
std::atomic<bool> perform_global_localization(false);

void signal_handler(int signal_num)
{
    exit_program = true;
}

void pointCloudIntegration(const geometry_msgs::Pose &estimated_pose) {
    if(lidar_buffer.empty()) return;
    // Transform point clouds in the LiDAR buffer to the actual estimated pose
    Eigen::Affine3d transform_initial_pose;
    tf::poseMsgToEigen(estimated_pose, transform_initial_pose);
    PointCloudXYZI::Ptr pc_aux(new PointCloudXYZI());
    bool first_time = true;
    for(int i=0; i < lidar_buffer.size(); i++) {
        PointCloudXYZI aux;
        pcl::transformPointCloud(*(lidar_buffer[i]), aux, transform_initial_pose);
        if(first_time) {
            *pc_aux = aux;
            first_time = false;
        }
        else *pc_aux += aux;
        if(i == lidar_buffer.size()-1)
            last_lidar_scan_in_local_map = aux;
    }
    local_scan = pc_aux;
}

void voxel_downsample(const PointCloudXYZI::Ptr &original_point_cloud, PointCloudXYZI::Ptr &point_cloud_filtered, float filter_size) {
    // Convert from PointCloudXYZI to pcl::toPCLPointCloud2 format
    pcl::PCLPointCloud2::Ptr pc_pcl2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pc_pcl2_filtered(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*original_point_cloud, *pc_pcl2);
    // Filter point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> grid_filter;
    grid_filter.setInputCloud(pc_pcl2);
    grid_filter.setLeafSize(filter_size, filter_size, filter_size);
    grid_filter.filter(*pc_pcl2_filtered);
    // Convert again to PointCloudXYZI format
    pcl::fromPCLPointCloud2(*pc_pcl2_filtered, *point_cloud_filtered);
}

void initializeMap(const sensor_msgs::PointCloud2ConstPtr &msg) {
    // Convert ROS point cloud to PCL data format
    pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr pcl_pc2_filtered(new pcl::PCLPointCloud2());
    pcl_conversions::toPCL(*msg, *pcl_pc2);

    // Downsample point cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> grid_filter;
    grid_filter.setInputCloud(pcl_pc2);
    grid_filter.setLeafSize(0.1, 0.1, 0.1);
    grid_filter.filter(*pcl_pc2_filtered);
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI);
    pcl::fromPCLPointCloud2(*pcl_pc2_filtered, *ptr);
    map_pcl_cloud = ptr;
}

void print4x4Matrix (const Eigen::Matrix4d & matrix) {
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

bool is_valid_transform(const geometry_msgs::Pose &current_pose, const geometry_msgs::Pose &optimized_pose) {
    // Calculate position difference
    Eigen::Vector3f optimized_position(optimized_pose.position.x, optimized_pose.position.y, optimized_pose.position.z);
    Eigen::Vector3f last_position(current_pose.position.x, current_pose.position.y, current_pose.position.z);
    if(std::sqrt(std::pow(optimized_position.x()-last_position.x(), 2) + std::pow(optimized_position.y()-last_position.y(), 2) + std::pow(optimized_position.z()-last_position.z(), 2)) > 2)
        return false;
    // Calculate orientation difference
    tf::Quaternion q_actual, q_last, q_delta;
    tf::quaternionMsgToTF(optimized_pose.orientation, q_actual);
    tf::quaternionMsgToTF(current_pose.orientation, q_actual);
    q_delta = q_actual * q_last.inverse();
    if(std::abs(q_delta.x()) > 0.38 || std::abs(q_delta.y()) > 0.38 || std::abs(q_delta.z()) > 0.38)
        return false;
    return true;
}

float icp_registration(const PointCloudXYZI::Ptr &source, const PointCloudXYZI::Ptr &target, Eigen::Matrix4d &transformation, PointCloudXYZI::Ptr &aligned_pc) {
    pcl::IterativeClosestPoint<PointType , PointType> icp;
    icp.setInputSource(source);
    icp.setInputTarget(target);
    icp.setMaximumIterations (20);

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    aligned_pc = ptr;
    icp.align(*aligned_pc);
    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        //std::cout << "\nICP transformation :" << std::endl;
        transformation = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
    }
    return icp.getFitnessScore();
}

bool global_localization(const geometry_msgs::Pose &estimated_pose, geometry_msgs::Pose &optimized_pose) {
    ROS_INFO("Initial pose: %.3f\t%.3f\t%.3f", estimated_pose.position.x, estimated_pose.position.y, estimated_pose.position.z);

    // Publish local scan
    sensor_msgs::PointCloud2 ros_cloud_msg;
    pcl::toROSMsg(*local_scan, ros_cloud_msg);
    ros_cloud_msg.header.stamp = ros::Time::now();
    ros_cloud_msg.header.frame_id = "camera_init";
    pub_concatenated_scan.publish(ros_cloud_msg);

    // Downsample map and local scan to bigger size
    PointCloudXYZI::Ptr filtered_map(new PointCloudXYZI());
    PointCloudXYZI::Ptr filtered_local_scan(new PointCloudXYZI());
    float map_filter_size, local_scan_filter_size;
    ros::param::get("/map_filter_size", map_filter_size);
    ros::param::get("/scan_filter_size", local_scan_filter_size);
    voxel_downsample(map_pcl_cloud, filtered_map, map_filter_size);
    voxel_downsample(local_scan, filtered_local_scan, local_scan_filter_size);

    // ICP Registration - register the local scan with the map, to get a first rough estimate
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    PointCloudXYZI::Ptr aligned_pc;
    float icp_fitness = icp_registration(local_scan, map_pcl_cloud, transformation_matrix, aligned_pc);
    if(icp_fitness < 0.6) {  // Valid registration
        // Create homogeneous transformation matrix for the transformation "estimated_pose -> map"
        Eigen::Affine3d initial_estimated_eigen;
        tf::poseMsgToEigen(estimated_pose, initial_estimated_eigen);
        // Transform translation matrix calculated by ICP to the Eigen::Affine3d format
        Eigen::Matrix3d rotation_matrix;
        for(int i=0; i < 3; i++) {
            for(int j=0; j < 3; j++)
                rotation_matrix(i,j) = transformation_matrix(i,j);
        }
        Eigen::Quaterniond q_rotation(rotation_matrix);
        Eigen::Affine3d first_optimized_affine;
        tf::Transform first_optimized_tf;
        first_optimized_tf.setOrigin(tf::Vector3(transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3)));
        first_optimized_tf.setRotation(tf::Quaternion(q_rotation.x(), q_rotation.y(), q_rotation.z(), q_rotation.w()));
        tf::transformTFToEigen(first_optimized_tf, first_optimized_affine);
        // Transform last received lidar scan to the new optimized pose
        pcl::transformPointCloud(last_lidar_scan_in_local_map, last_lidar_scan_in_local_map, first_optimized_affine);
        // ICP Registration - register the last received lidar scan with the local scan
        PointCloudXYZI::Ptr aligned_pc2, last_lidar_scan_ptr(new PointCloudXYZI());
        *last_lidar_scan_ptr = last_lidar_scan_in_local_map;
        icp_fitness = icp_registration(last_lidar_scan_ptr, aligned_pc, transformation_matrix, aligned_pc2);
        if(icp_fitness < 0.015) { // Valid registration
            // Transform translation matrix calculated by ICP to the Eigen::Affine3d format
            Eigen::Affine3d second_optimized_affine;
            tf::Transform second_optimized_tf;
            for(int i=0; i < 3; i++) {
                for(int j=0; j < 3; j++)
                    rotation_matrix(i,j) = transformation_matrix(i,j);
            }
            Eigen::Quaterniond q_rotation(rotation_matrix);
            second_optimized_tf.setOrigin(tf::Vector3(transformation_matrix(0, 3), transformation_matrix(1, 3), transformation_matrix(2, 3)));
            second_optimized_tf.setRotation(tf::Quaternion(q_rotation.x(), q_rotation.y(), q_rotation.z(), q_rotation.w()));
            tf::transformTFToEigen(second_optimized_tf, second_optimized_affine);

            // Calculate the optimized pose of the robot
            Eigen::Affine3d optimized_affine = initial_estimated_eigen * first_optimized_affine * second_optimized_affine;
            Eigen::Matrix4d optimized_matrix = optimized_affine.matrix();
            for(int i=0; i < 3; i++) {
                for(int j=0; j < 3; j++)
                    rotation_matrix(i,j) = optimized_matrix(i,j);
            }
            Eigen::Quaterniond q_new_orientation(rotation_matrix);
            // Fill msg to publish with the optimized pose
            geometry_msgs::Pose optimized_pose_msg;
            optimized_pose_msg.position.x = optimized_matrix(0,3);
            optimized_pose_msg.position.y = optimized_matrix(1,3);
            optimized_pose_msg.position.z = optimized_matrix(2,3);
            optimized_pose_msg.orientation.x = q_new_orientation.x();
            optimized_pose_msg.orientation.y = q_new_orientation.y();
            optimized_pose_msg.orientation.z = q_new_orientation.z();
            optimized_pose_msg.orientation.w = q_new_orientation.w();
            if(is_valid_transform(estimated_pose, optimized_pose_msg)) {
                optimized_pose = optimized_pose_msg;
                return true;
            }
            else {
                return false;
            }
        }
        else {
            ROS_WARN("Pose didn't converged! Try again...");
            return false;
        }
    }
    else {
        ROS_WARN("Pose didn't converged! Try again...");
        return false;
    }
}

bool srv_optimize_pose(fast_lio::OptimizePose::Request &req, fast_lio::OptimizePose::Response &res) {
    bool valid_estimation;
    geometry_msgs::PoseWithCovarianceStampedConstPtr initial_pose;
    geometry_msgs::Pose current_pose, optimized_pose;
    ROS_INFO("Received request to estimate global localization!");

    perform_global_localization = true;
    // Clear LiDAR buffer
    lidar_buffer.clear();
    // Wait until filling the LiDAR buffer with enough data
    while(lidar_buffer.size() < NUM_SCANS_TO_CONCATENATE) {
        ros::Duration(0.05).sleep();
    }

    if(req.need_initial_estimate) needed_initial_estimate = true;
    else needed_initial_estimate = false;

    do {
        if(needed_initial_estimate) {
            initial_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");
            current_pose = initial_pose->pose.pose;
        }
        else {
            current_pose = req.current_pose;
        }
        // Create denser point cloud
        mtx_lidar.lock();
        pointCloudIntegration(current_pose);
        mtx_lidar.unlock();
        // Estimate the global localization, performing ICP registration between the local scan and the map
        valid_estimation = global_localization(current_pose, optimized_pose);
        if(!valid_estimation) {
            needed_initial_estimate = true;
            ROS_WARN("Failed to estimate global localization! Please set a rough estimation of the robot's pose...");
        }
    } while(!valid_estimation);


    res.optimized_pose = optimized_pose;
    perform_global_localization = false;
    return true;
}

void cbLidarScan(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    if(!perform_global_localization) return;
    if(mtx_lidar.try_lock()) {
        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        // Publish local scan
        sensor_msgs::PointCloud2 ros_cloud_msg;
        pcl::toROSMsg(*ptr, ros_cloud_msg);
        ros_cloud_msg.header.stamp = ros::Time::now();
        ros_cloud_msg.header.frame_id = "camera_init";
        pub_registered_scan.publish(ros_cloud_msg);
        if(lidar_buffer.size() < NUM_SCANS_TO_CONCATENATE) {
            lidar_buffer.push_back(ptr);
            last_lidar_scan_time = msg->header.stamp.toSec();
        }
        else {
            lidar_buffer.pop_front();
            lidar_buffer.push_back(ptr);
        }
        mtx_lidar.unlock();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "global_localization_node");
    ros::NodeHandle nh;

    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    ros::param::set("/map_filter_size", 0.3);
    ros::param::set("/scan_filter_size", 0.1);

    sub_lidar_scan = nh.subscribe("/livox/lidar", 10, cbLidarScan);
    ros::ServiceServer service = nh.advertiseService("global_localization", srv_optimize_pose);

    pub_concatenated_scan = nh.advertise<sensor_msgs::PointCloud2>("/concatenated", 10);
    pub_registered_scan = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 10);

    sensor_msgs::PointCloud2ConstPtr map_cloud;
    map_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map");
    initializeMap(map_cloud);
    ROS_INFO("Map initialized!");

    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin();

    return 0;
}

// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <stdio.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf_conversions/tf_eigen.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "fast_lio/OptimizePose.h"
#include <std_msgs/Int8.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cvstd.inl.hpp>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)
#define NUM_SCANS_TO_CONCATENATE 10

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
bool need_global_localization = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer, mtx_odom, mtx_lidar;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_camera = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, imu_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool map_initialized = false;
bool initial_pose_defined = false;
bool use_only_visual_odom = false;
bool vio_can_start = false;
bool robot_lost = true;
bool need_set_new_initial_pose = true;
bool perform_only_localization = false;
bool use_tracking_camera = true;
bool valid_camera_data = true;
bool tf_tree_initialized = false;
bool initial_tf_set = false;

geometry_msgs::TwistStamped last_camera_vel_msg;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<nav_msgs::Odometry::ConstPtr> camera_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

PointCloudXYZI::Ptr map_cloud(new PointCloudXYZI());
PointCloudXYZI::Ptr local_scan;
PointCloudXYZI last_lidar_scan_in_local_map;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

uint8_t res_counter = 0;

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point, last_state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;
geometry_msgs::Pose actual_camera_pose, last_camera_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

ros::Publisher pubLaserCloudFull;
ros::Publisher pub_desired_velocity;
ros::Publisher pub_robot_state;

tf2_ros::Buffer tfBuffer;
tf::Transform initial_tf;

void publish_point_cloud_world(const ros::Publisher & pubLaserCloudFull, const PointCloudXYZI::Ptr &pc);

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    if(perform_only_localization && robot_lost) {
        if(mtx_lidar.try_lock()) {
            PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
            p_pre->process(msg, ptr);
            // Publish local scan
            publish_point_cloud_world(pubLaserCloudFull, ptr);
            if(lidar_buffer.size() < NUM_SCANS_TO_CONCATENATE) {
                lidar_buffer.push_back(ptr);
            }
            else {
                lidar_buffer.pop_front();
                lidar_buffer.push_back(ptr);
            }
            mtx_lidar.unlock();
        }
    }
    else {
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = msg->header.stamp.toSec();

        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }

        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);

        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    if(robot_lost) return;
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    msg->linear_acceleration.x /= 9.81;
    msg->linear_acceleration.y /= 9.81;
    msg->linear_acceleration.z /= 9.81;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void initialize_map(const sensor_msgs::PointCloud2ConstPtr &msg) {
    pcl::fromROSMsg(*msg, *map_cloud);
    downSizeFilterSurf.setInputCloud(map_cloud);
    downSizeFilterSurf.filter(*map_cloud);
    ikdtree.Build(map_cloud->points);
}

void camera_odom_cbk(const nav_msgs::OdometryConstPtr &msg_in) {
    if(perform_only_localization && robot_lost) return;
    if(isnan(msg_in->pose.pose.position.x) || isnan(msg_in->pose.pose.position.y) || isnan(msg_in->pose.pose.position.z)) {
        ROS_ERROR("Invalid message");
        valid_camera_data = false;
        return;
    }
    valid_camera_data = true;
    static tf2_ros::TransformBroadcaster br;
    static tf2_ros::StaticTransformBroadcaster static_br;
    if (!tf_tree_initialized) {
        tf_tree_initialized = true;
        // Create world frame
        geometry_msgs::TransformStamped camera_odom_to_lidar_transform;
        camera_odom_to_lidar_transform = tfBuffer.lookupTransform("camera_odom_frame", "LiDAR_frame", ros::Time(0),
                                                                  ros::Duration(5.0));

        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = msg_in->header.stamp;
        static_transformStamped.header.frame_id = "camera_odom_frame";
        static_transformStamped.child_frame_id = "camera_init";
        static_transformStamped.transform = camera_odom_to_lidar_transform.transform;
        static_br.sendTransform(static_transformStamped);

        // Initialize world -> base_link frame
        geometry_msgs::TransformStamped transform_msg;
        transform_msg.header.stamp = msg_in->header.stamp;
        transform_msg.header.frame_id = "camera_init";
        transform_msg.child_frame_id = "body";
        transform_msg.transform.translation.x = 0;
        transform_msg.transform.translation.y = 0;
        transform_msg.transform.translation.z = 0;
        transform_msg.transform.rotation.x = 0;
        transform_msg.transform.rotation.y = 0;
        transform_msg.transform.rotation.z = 0;
        transform_msg.transform.rotation.w = 1;
        br.sendTransform(transform_msg);
    }
    nav_msgs::Odometry::Ptr msg(new nav_msgs::Odometry(*msg_in));
    if(perform_only_localization && initial_tf_set) {
        tf::Transform odom_tf;
        // Fill odom_tf with odometry data
        odom_tf.setOrigin(tf::Vector3(msg_in->pose.pose.position.x, msg_in->pose.pose.position.y, msg_in->pose.pose.position.z));
        odom_tf.setRotation(tf::Quaternion(msg_in->pose.pose.orientation.x, msg_in->pose.pose.orientation.y, msg_in->pose.pose.orientation.z, msg_in->pose.pose.orientation.w));
        odom_tf = initial_tf * odom_tf;
        // Update new transformed odometry data
        msg->pose.pose.position.x = odom_tf.getOrigin().x();
        msg->pose.pose.position.y = odom_tf.getOrigin().y();
        msg->pose.pose.position.z = odom_tf.getOrigin().z();
        msg->pose.pose.orientation.x = odom_tf.getRotation().x();
        msg->pose.pose.orientation.y = odom_tf.getRotation().y();
        msg->pose.pose.orientation.z = odom_tf.getRotation().z();
        msg->pose.pose.orientation.w = odom_tf.getRotation().w();
    }
    double timestamp = msg->header.stamp.toSec();
    mtx_odom.lock();
    if (timestamp < last_timestamp_camera)
    {
        ROS_WARN("Camera odometry loop back, clear buffer");
        camera_buffer.clear();
    }
    last_timestamp_camera = timestamp;
    camera_buffer.push_back(msg);
    mtx_odom.unlock();
    // Publish world -> camera_pose frame
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = msg_in->header.stamp;
    transform_msg.header.frame_id = "camera_init";
    transform_msg.child_frame_id = "camera_pose";
    transform_msg.transform.translation.x = msg->pose.pose.position.x;
    transform_msg.transform.translation.y = msg->pose.pose.position.y;
    transform_msg.transform.translation.z = msg->pose.pose.position.z;
    transform_msg.transform.rotation.x = msg->pose.pose.orientation.x;
    transform_msg.transform.rotation.y = msg->pose.pose.orientation.y;
    transform_msg.transform.rotation.z = msg->pose.pose.orientation.z;
    transform_msg.transform.rotation.w = msg->pose.pose.orientation.w;
    br.sendTransform(transform_msg);
}

bool get_camera_lidar_pose_in_world_frame(geometry_msgs::Pose &result)
{
    geometry_msgs::TransformStamped lidar_pose_transform;
    if(!tfBuffer.canTransform("camera_init", "LiDAR_frame", ros::Time(lidar_end_time), ros::Duration(1.0)))
        return false;
    lidar_pose_transform = tfBuffer.lookupTransform("camera_init", "LiDAR_frame", ros::Time(lidar_end_time), ros::Duration(0.3));
    // Save transformed pose
    result.position.x = lidar_pose_transform.transform.translation.x;
    result.position.y = lidar_pose_transform.transform.translation.y;
    result.position.z = lidar_pose_transform.transform.translation.z;
    result.orientation.x = lidar_pose_transform.transform.rotation.x;
    result.orientation.y = lidar_pose_transform.transform.rotation.y;
    result.orientation.z = lidar_pose_transform.transform.rotation.z;
    result.orientation.w = lidar_pose_transform.transform.rotation.w;
    //ROS_INFO("Lidar Pose in world: %.3f\t%.3f\t%.3f\n", result.position.x, result.position.y, result.position.z);
    return true;
}

bool get_camera_delta_odom(geometry_msgs::Pose &result)
{
    if(!get_camera_lidar_pose_in_world_frame(actual_camera_pose)) return false;
    // Calculate delta position
    result.position.x = actual_camera_pose.position.x - last_camera_pose.position.x;
    result.position.y = actual_camera_pose.position.y - last_camera_pose.position.y;
    result.position.z = actual_camera_pose.position.z - last_camera_pose.position.z;
    // Calculate delta orientation
    tf2::Quaternion q_last_orientation, q_actual_orientation, q_delta_orientation;
    tf2::fromMsg(last_camera_pose.orientation, q_last_orientation);
    tf2::fromMsg(actual_camera_pose.orientation, q_actual_orientation);
    q_delta_orientation = q_actual_orientation * q_last_orientation.inverse();
    result.orientation = tf2::toMsg(q_delta_orientation);

    // Update last pose
    last_camera_pose = actual_camera_pose;
    return true;
}



double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty() || (valid_camera_data && camera_buffer.empty() && use_tracking_camera)) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    if(!imu_pushed) {
        /*** push imu data, and pop from imu buffer ***/
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        if(imu_buffer.empty() && (imu_time < lidar_end_time)) return false;
        imu_pushed = true;
    }

    double camera_time;
    if(use_tracking_camera && valid_camera_data) {
        while (!camera_buffer.empty())
        {
            camera_time = camera_buffer.front()->header.stamp.toSec();
            if(camera_time >= lidar_end_time) {
                last_camera_vel_msg.header = camera_buffer.front()->header;
                last_camera_vel_msg.header.frame_id = "camera_pose_frame";
                last_camera_vel_msg.twist = camera_buffer.front()->twist.twist;
                camera_buffer.pop_front();
                break;
            }
            camera_buffer.pop_front();
        }
        if(camera_buffer.empty() && camera_time < lidar_end_time)
            return false;
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    imu_pushed = false;
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

void publish_point_cloud_world(const ros::Publisher & pubLaserCloudFull, const PointCloudXYZI::Ptr &pc)
{
    int size = pc->points.size();
    PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyToWorld(&pc->points[i], \
                                &laserCloudWorld->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFull.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time::now();// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, ros::Time::now(), "camera_init", "body" ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void publish_robot_state(int8_t state) // state = 0 -> lost; state = 1 -> normal
{
    if(state < 0 || state > 1) return;
    std_msgs::Int8 msg;
    msg.data = state;
    pub_robot_state.publish(msg);
}

void publish_velocity_message(float vlin, float omega)
{
    if(std::abs(vlin) > 1.0 || std::abs(omega) > 0.7) return;
    geometry_msgs::Twist msg;
    msg.linear.x = vlin;
    msg.angular.x = omega;
    msg.angular.z = 3; // keep latest gait type
    pub_desired_velocity.publish(msg);
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

void print4x4Matrix (const Eigen::Matrix4d & matrix) {
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
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

bool global_localization(const geometry_msgs::Pose &estimated_pose, geometry_msgs::Pose &optimized_pose)
{
    // Downsample map and local scan to bigger size
    PointCloudXYZI::Ptr filtered_local_scan(new PointCloudXYZI());
    float local_scan_filter_size;
    ros::param::get("/scan_filter_size", local_scan_filter_size);
    voxel_downsample(local_scan, filtered_local_scan, local_scan_filter_size);

    // ICP Registration - register the local scan with the map, to get a first rough estimate
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
    PointCloudXYZI::Ptr aligned_pc;
    float icp_fitness = icp_registration(local_scan, map_cloud, transformation_matrix, aligned_pc);
    if(icp_fitness < 1.5) {  // Valid registration
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

void recovery_module()
{
    geometry_msgs::Pose current_pose, optimized_pose;
    bool valid_estimation;
    // Clear LiDAR buffer
    lidar_buffer.clear();
    // Wait until filling the LiDAR buffer with enough data
    while(lidar_buffer.size() < NUM_SCANS_TO_CONCATENATE) {
        ros::Duration(0.05).sleep();
    }
    do {
        // Set current pose
        if(need_set_new_initial_pose) {
            geometry_msgs::PoseWithCovarianceStampedConstPtr initial_pose;
            initial_pose = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("/initialpose");
            current_pose = initial_pose->pose.pose;
        }
        else {
            current_pose.position.x = state_point.pos(0);
            current_pose.position.y = state_point.pos(1);
            current_pose.position.z = state_point.pos(2);
            current_pose.orientation.x = state_point.rot.coeffs()[0];
            current_pose.orientation.y = state_point.rot.coeffs()[1];
            current_pose.orientation.z = state_point.rot.coeffs()[2];
            current_pose.orientation.w = state_point.rot.coeffs()[3];
        }
        // Create denser point cloud
        mtx_lidar.lock();
        pointCloudIntegration(current_pose);
        mtx_lidar.unlock();
        // Estimate the global localization, performing ICP registration between the local scan and the map
        valid_estimation = global_localization(current_pose, optimized_pose);
        if(!valid_estimation) {
            need_set_new_initial_pose = true;
            ROS_WARN("Failed to estimate global localization! Please set a rough estimation of the robot's pose...");
        }
    } while(!valid_estimation);

    // Use the new optimized pose
    state_point.pos(0) = optimized_pose.position.x;
    state_point.pos(1) = optimized_pose.position.y;
    state_point.pos(2) = optimized_pose.position.z;
    state_point.rot.coeffs()[0] = optimized_pose.orientation.x;
    state_point.rot.coeffs()[1] = optimized_pose.orientation.y;
    state_point.rot.coeffs()[2] = optimized_pose.orientation.z;
    state_point.rot.coeffs()[3] = optimized_pose.orientation.w;
    ROS_INFO("New recovered position: %.3f\t%.3f\t%.3f", state_point.pos(0), state_point.pos(1), state_point.pos(2));
    kf.change_x(state_point);
    if(!initial_tf_set) {
        initial_tf_set = true;
        initial_tf.setOrigin(tf::Vector3(state_point.pos(0), state_point.pos(1), state_point.pos(2)));
        initial_tf.setRotation(tf::Quaternion(state_point.rot.coeffs()[0], state_point.rot.coeffs()[1], state_point.rot.coeffs()[2], state_point.rot.coeffs()[3]));
        // Initialize last camera position
        last_camera_pose.position.x = state_point.pos(0);
        last_camera_pose.position.y = state_point.pos(1);
        last_camera_pose.position.z = state_point.pos(2);
        last_camera_pose.orientation.x = state_point.rot.coeffs()[0];
        last_camera_pose.orientation.y = state_point.rot.coeffs()[1];
        last_camera_pose.orientation.z = state_point.rot.coeffs()[2];
        last_camera_pose.orientation.w = state_point.rot.coeffs()[3];
    }
    last_state_point = state_point;
    need_set_new_initial_pose = false;
    publish_robot_state(1); // Normal mode
    robot_lost = false;
    res_counter = 0;
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

    /** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 3 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        use_only_visual_odom = true;
        res_counter++;
        if(res_counter > 20 && perform_only_localization) robot_lost = true;
        ROS_WARN("No Effective Points... \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;

    if (effct_feat_num < 50 && perform_only_localization)
    {
        ekfom_data.valid = false;
        use_only_visual_odom = true;
        if(res_mean_last > 0.1 && perform_only_localization) res_counter++;
        if(res_counter > 20 && perform_only_localization) robot_lost = true;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    if(res_mean_last > 0.1) {
        use_only_visual_odom = true;
        if(perform_only_localization) {
            res_counter++;
            if(res_counter > 20) {
                ekfom_data.valid = false;
                robot_lost = true;
                res_counter = 0;
                return;
            }
        }
    }

    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();

//    ROS_INFO("Features = %d\tResiduo = %.3f", effct_feat_num, res_mean_last);

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    tf2_ros::TransformListener tfListener(tfBuffer);

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<bool>("perform_only_localization", perform_only_localization, false);
    nh.param<bool>("use_tracking_camera", use_tracking_camera, true);
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;

    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    last_state_point = kf.get_x();
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_camera_odom = nh.subscribe("/tracking_camera/odom/sample", 100, camera_odom_cbk);
    pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/Odometry", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path", 100000);
    pub_desired_velocity = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_robot_state = nh.advertise<std_msgs::Int8>("/robot_state", 1);
//------------------------------------------------------------------------------------------------------

    // Initialize map
    if(perform_only_localization) {
        sensor_msgs::PointCloud2ConstPtr map_msg;
        map_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/map");
        initialize_map(map_msg);
        ROS_INFO("Map initialized!");
        robot_lost = true;
        initial_tf_set = false;
    }
    else {
        initial_tf_set = true;
        robot_lost = false;
    }

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    uint8_t counter = 0;
    while (status)
    {
        if (flg_exit) break;
        if(perform_only_localization && robot_lost) {
            lidar_buffer.clear();
            time_buffer.clear();
            imu_buffer.clear();
            camera_buffer.clear();
            publish_robot_state(0); // Lost state
            // Send ROS message to stop the robot
            publish_velocity_message(0, 0);
            // Call the recovery module to get a new valid pose for the robot
            std::thread thread_recovery_pose(recovery_module);
            while(robot_lost) {
                ros::spinOnce();
                ros::Duration(0.005).sleep();
            }
            thread_recovery_pose.join();
            ros::getGlobalCallbackQueue()->clear();
            p_imu->Reset();
            lidar_buffer.clear();
            time_buffer.clear();
            imu_buffer.clear();
            camera_buffer.clear();
        }
        ros::spinOnce();
        if(sync_packages(Measures))
        {
//            ROS_INFO("Sync packages");
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            // Update IMU guess with camera odometry
            geometry_msgs::Pose camera_delta_odom;
            if(use_tracking_camera && valid_camera_data) {
                if(get_camera_delta_odom(camera_delta_odom)) {
                    state_point.pos(0) = last_state_point.pos(0) + camera_delta_odom.position.x;
                    state_point.pos(1) = last_state_point.pos(1) + camera_delta_odom.position.y;
                    state_point.pos(2) = last_state_point.pos(2) + camera_delta_odom.position.z;
                    tf2::Quaternion q_delta_rotation, q_new_rotation;
                    tf2::Quaternion q_last_rotation(last_state_point.rot.coeffs()[0], last_state_point.rot.coeffs()[1], last_state_point.rot.coeffs()[2], last_state_point.rot.coeffs()[3]);
                    tf2::fromMsg(camera_delta_odom.orientation, q_delta_rotation);
                    q_new_rotation = q_delta_rotation * q_last_rotation;
                    geometry_msgs::Quaternion result = tf2::toMsg(q_new_rotation);
                    state_point.rot.coeffs()[0] = result.x;
                    state_point.rot.coeffs()[1] = result.y;
                    state_point.rot.coeffs()[2] = result.z;
                    state_point.rot.coeffs()[3] = result.w;
                    // Transform velocity to the body frame
                    geometry_msgs::PointStamped p_in, p_transformed;
                    p_in.header = last_camera_vel_msg.header;
                    p_in.point.x = last_camera_vel_msg.twist.linear.x;
                    p_in.point.y = last_camera_vel_msg.twist.linear.y;
                    p_in.point.z = last_camera_vel_msg.twist.linear.z;
                    tfBuffer.transform(p_in, p_transformed, "LiDAR_frame");
                    tf::Transform vel_tf;
                    vel_tf.setOrigin(tf::Vector3(p_transformed.point.x, p_transformed.point.y, p_transformed.point.z));
                    vel_tf.setRotation(tf::Quaternion(0, 0, 0, 1));
                    vel_tf = initial_tf * vel_tf;
                    state_point.vel(0) = vel_tf.getOrigin().x();
                    state_point.vel(1) = vel_tf.getOrigin().y();
                    state_point.vel(2) = vel_tf.getOrigin().z();
                    kf.change_x(state_point);
                }
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan...\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;

            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            last_state_point = state_point;

            double t_update_end = omp_get_wtime();

            use_only_visual_odom = false;

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            if(!perform_only_localization)
                map_incremental();
            t5 = omp_get_wtime();

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en && !perform_only_localization)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}

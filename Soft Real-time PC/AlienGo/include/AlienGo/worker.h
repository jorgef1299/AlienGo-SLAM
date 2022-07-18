#ifndef SRC_WORKER_H
#define SRC_WORKER_H

#include <QWidget>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class TWorker: public QObject
{
    Q_OBJECT
public:
    ros::CallbackQueue* getCallbackQueue();
    // ROS Callbacks
    void cbProcessTopCameraFrame(const sensor_msgs::ImageConstPtr& ros_image);
    void cbProcessBottomCameraFrame(const sensor_msgs::ImageConstPtr& ros_image);

public slots:
    void run();
private:
    boost::shared_ptr<ros::AsyncSpinner> FSpinner;
    ros::CallbackQueue images_queue;
    // Other functions
    void convert_depth_to_color(const sensor_msgs::ImageConstPtr &msg, QImage& qt_image, uint16_t min_depth, uint16_t max_depth);
    Q_SIGNALS:
    void topCameraFrameReadyToShow(const QPixmap& qt_pixmap);
    void bottomCameraFrameReadyToShow(const QPixmap& qt_pixmap);
};

#endif //SRC_WORKER_H

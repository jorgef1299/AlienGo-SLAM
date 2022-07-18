#include "worker.h"

void TWorker::run()
{
    ros::MultiThreadedSpinner spinner2(2);
    spinner2.spin(&images_queue);
}

void TWorker::cbProcessTopCameraFrame(const sensor_msgs::ImageConstPtr& ros_image)
{
    QImage qt_image;
    if(ros_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        convert_depth_to_color(ros_image, qt_image, 500, 30000);
    }
    else {
        qt_image = QImage(&ros_image->data[0], ros_image->width, ros_image->height, ros_image->step, QImage::Format_RGB888);
    }
    QPixmap qt_pixmap = QPixmap::fromImage(qt_image);
    emit topCameraFrameReadyToShow(qt_pixmap);
}

void TWorker::cbProcessBottomCameraFrame(const sensor_msgs::ImageConstPtr &ros_image)
{
    QImage qt_image = QImage(&ros_image->data[0], ros_image->width, ros_image->height, ros_image->step, QImage::Format_RGB888);
    QPixmap qt_pixmap = QPixmap::fromImage(qt_image);
    emit bottomCameraFrameReadyToShow(qt_pixmap);
}


ros::CallbackQueue* TWorker::getCallbackQueue() {
    return &images_queue;
}
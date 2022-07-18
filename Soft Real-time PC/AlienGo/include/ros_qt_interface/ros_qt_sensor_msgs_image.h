#ifndef ROS_QT_SENSOR_MSGS_IMAGE_H
#define ROS_QT_SENSOR_MSGS_IMAGE_H

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include <QObject>
#include <QMutex>

namespace ros_qt_interface {

    class TRosQtSensorMsgsImagePub {
        public:
            TRosQtSensorMsgsImagePub(std::string TopicName, std::string Namespace = "~", unsigned int queueSize = 10,
                                     bool latch = false);
            virtual ~TRosQtSensorMsgsImagePub();

            void SendData(const sensor_msgs::Image &data);

        private:
            image_transport::ImageTransport* it;
            ros::NodeHandle FNodeHandle;
            image_transport::Publisher FDataPub;
    };

    class TRosQtSensorMsgsImageSub : public QObject {
        Q_OBJECT
        public:
            TRosQtSensorMsgsImageSub(std::string TopicName, std::string Namespace = "~", unsigned int queueSize = 10);
            virtual ~TRosQtSensorMsgsImageSub();
            void GetData(sensor_msgs::Image &data);
            void StopSubscriber();

        Q_SIGNALS:
            void DataReceived();

        private:
            QMutex FMutex;
            ros::NodeHandle FNodeHandle;
            image_transport::ImageTransport* it;
            image_transport::Subscriber FDataSub;
            sensor_msgs::Image FData;
            void DataReceivedCallback(const sensor_msgs::ImageConstPtr& data);
    };
}

#endif //ROS_QT_SENSOR_MSGS_IMAGE_H

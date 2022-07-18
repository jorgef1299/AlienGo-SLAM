#include "../../include/ros_qt_interface/ros_qt_sensor_msgs_image.h"

namespace ros_qt_interface
{

    TRosQtSensorMsgsImagePub::TRosQtSensorMsgsImagePub(std::string TopicName, std::string Namespace, unsigned int queueSize, bool latch) :
        FNodeHandle(Namespace)
    {
        it = new image_transport::ImageTransport(FNodeHandle);
        FDataPub = it->advertise(TopicName, queueSize, latch);
    }

    void TRosQtSensorMsgsImagePub::SendData(const sensor_msgs::Image &data)
    {
        FDataPub.publish(data);
    }

    TRosQtSensorMsgsImagePub::~TRosQtSensorMsgsImagePub()
    {
        delete it;
    }

    TRosQtSensorMsgsImageSub::TRosQtSensorMsgsImageSub(std::string TopicName, std::string Namespace, unsigned int queueSize) :
        FNodeHandle(Namespace)
    {
        it = new image_transport::ImageTransport(FNodeHandle);
        FDataSub = it->subscribe(TopicName, queueSize, &TRosQtSensorMsgsImageSub::DataReceivedCallback,this);
    }

    TRosQtSensorMsgsImageSub::~TRosQtSensorMsgsImageSub()
    {

    }

    void TRosQtSensorMsgsImageSub::GetData(sensor_msgs::Image &data)
    {
        FMutex.lock();
        data=FData;
        FMutex.unlock();
    }

    void TRosQtSensorMsgsImageSub::StopSubscriber()
    {
        FDataSub.shutdown();
    }

    void TRosQtSensorMsgsImageSub::DataReceivedCallback(const sensor_msgs::ImageConstPtr& data)
    {
        FMutex.lock();
        FData=*data;
        FMutex.unlock();
        Q_EMIT DataReceived();
    }

}


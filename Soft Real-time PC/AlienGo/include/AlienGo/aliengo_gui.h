#ifndef SRC_ALIENGO_GUI_H
#define SRC_ALIENGO_GUI_H

// QT includes
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QRadioButton>
#include <QThread>
#include <QCheckBox>

// RVIZ includes
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

// ROS includes
#include <ros/ros.h>
#include "ros_qt_sensor_msgs_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

// Worker class
#include "worker.h"

namespace Ui {
    class MainWindow;
}

namespace Aliengo {

    enum class CameraState {
        Disabled,
        RGB,
        Depth
    };

    enum class MapState {
        Disabled,
        Two_D,
        Three_D
    };

    class MainWindow: public QMainWindow
    {
    Q_OBJECT
    public:
        MainWindow(QWidget* parent=0);
        virtual ~MainWindow();
    private Q_SLOTS:
        void RadioButtonTopCameraPressed(int button_id);
        void RadioButtonBottomCameraPressed(int button_id);
        void RadioButtonMapPressed(int button_id);
        void SLOT_UpdateTopCameraImage(const QPixmap& qt_pixmap);
        void SLOT_UpdateBottomCameraImage(const QPixmap& qt_pixmap);
        void SLOT_CheckboxCurrentPoseTF(bool state);
        void SLOT_CheckboxOdometry(bool state);
        void SLOT_CheckboxPlannedTrajectory(bool state);
    private:
        rviz::VisualizationManager* FManager;
        rviz::RenderPanel* FRender_panel;
        rviz::Display* FDisplayGrid;
        rviz::Display* FDisplayMap;
        rviz::Display* FDisplayOdometry;
        rviz::Display* FDisplayTF;
        Ui::MainWindow *ui;
        CameraState FTopCameraState, FBottomCameraState;
        MapState FMapState;
        QThread FWorker_thread;
        TWorker* FWorker;
        ros::NodeHandle n_private;
        image_transport::ImageTransport* it;
        image_transport::Subscriber FTopCameraImageSub;
        image_transport::Subscriber FBottomCameraImageSub;
        // ROS parameters
        std::string FTopCameraRGBTopicName, FBottomCameraRGBTopicName;
        std::string FTopCameraDepthTopicName, FBottomCameraDepthTopicName;
        int FTopCameraMinDepth, FTopCameraMaxDepth;
        int FBottomCameraMinDepth, FBottomCameraMaxDepth;
        std::string FMap3DTopicName, FMap2DTopicName;
        std::string FFrameId;
        int FDisplayPointCloud2SizePixels, FDisplayPointCloud2DecayTime;
        std::string FOdometryTopicName;
        float FOdometryPositionTolerance, FOdometryAngleTolerance;
        int FOdometrySamplesToKeep;
        float FOdometryShapeAlpha, FOdometryShapeShaftLength, FOdometryShapeHeadLength;
    };
}

#endif //SRC_ALIENGO_GUI_H

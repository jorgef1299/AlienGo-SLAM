#include "aliengo_gui.h"
#include "ui_mainwindow.h"

namespace Aliengo {
    MainWindow::MainWindow(QWidget *parent) :
            QMainWindow(parent), ui(new Ui::MainWindow)
    {
        // Load ROS Parameters
        ros::param::get("/top_camera/rgb_image_topic", FTopCameraRGBTopicName);
        ros::param::get("/top_camera/depth_image_topic", FTopCameraDepthTopicName);
        ros::param::get("/top_camera/colorize_min_depth", FTopCameraMinDepth);
        ros::param::get("/top_camera/colorize_max_depth", FTopCameraMaxDepth);
        ros::param::get("/bottom_camera/rgb_image_topic", FBottomCameraRGBTopicName);
        ros::param::get("/bottom_camera/depth_image_topic", FBottomCameraDepthTopicName);
        ros::param::get("/bottom_camera/colorize_min_depth", FBottomCameraMinDepth);
        ros::param::get("/bottom_camera/colorize_max_depth", FBottomCameraMaxDepth);
        ros::param::get("/map/3d_topic", FMap3DTopicName);
        ros::param::get("/map/2d_topic", FMap2DTopicName);
        ros::param::get("/frame_id", FFrameId);
        ros::param::get("/display_pointcloud2/size_pixels", FDisplayPointCloud2SizePixels);
        ros::param::get("/display_pointcloud2/decay_time", FDisplayPointCloud2DecayTime);
        ros::param::get("/odometry/topic", FOdometryTopicName);
        ros::param::get("/odometry/position_tolerance", FOdometryPositionTolerance);
        ros::param::get("/odometry/angle_tolerance", FOdometryAngleTolerance);
        ros::param::get("/odometry/samples_to_keep", FOdometrySamplesToKeep);
        ros::param::get("/odometry/shape/alpha", FOdometryShapeAlpha);
        ros::param::get("/odometry/shape/shaft_length", FOdometryShapeShaftLength);
        ros::param::get("/odometry/shape/head_length", FOdometryShapeHeadLength);

        // Load UI
        ui->setupUi(this);

        // Group Radio buttons in groups
        QButtonGroup* button_group_top_camera = new QButtonGroup();
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_1);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_2);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_3);
        QButtonGroup* button_group_bottom_camera = new QButtonGroup();
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_1);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_2);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_3);
        QButtonGroup* button_group_map = new QButtonGroup();
        button_group_map->addButton(ui->radioButton_Map_1);
        button_group_map->addButton(ui->radioButton_Map_2);
        button_group_map->addButton(ui->radioButton_Map_3);

        QVBoxLayout* new_vbox = new QVBoxLayout();
        FRender_panel = new rviz::RenderPanel();
        new_vbox->addWidget(FRender_panel);
        ui->GroupBox_Map->layout()->addWidget(FRender_panel);

        FWorker = new TWorker;
        FWorker->moveToThread(&FWorker_thread);

        FManager = new rviz::VisualizationManager(FRender_panel);
        FRender_panel->initialize(FManager->getSceneManager(), FManager);
        FManager->initialize();
        FManager->startUpdate();
        FManager->setFixedFrame(FFrameId.c_str());
        FDisplayGrid = FManager->createDisplay("rviz/Grid", "adjustable grid", true);

        connect(button_group_top_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonTopCameraPressed(int)));
        connect(button_group_bottom_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonBottomCameraPressed(int)));
        connect(button_group_map, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonMapPressed(int)));
        connect(&FWorker_thread, SIGNAL(started()), FWorker, SLOT(run()));
        connect(FWorker, SIGNAL(topCameraFrameReadyToShow(const QPixmap &)), this, SLOT(SLOT_UpdateTopCameraImage(const QPixmap&)));
        connect(FWorker, SIGNAL(bottomCameraFrameReadyToShow(const QPixmap &)), this, SLOT(SLOT_UpdateBottomCameraImage(const QPixmap&)));
        connect(ui->actionCurrentPoseTF, SIGNAL(triggered(bool)), this, SLOT(SLOT_CheckboxCurrentPoseTF(bool)));
        connect(ui->actionOdometry, SIGNAL(triggered(bool)), this, SLOT(SLOT_CheckboxOdometry(bool)));
        connect(ui->actionPlanned_Trajectory, SIGNAL(triggered(bool)), this, SLOT(SLOT_CheckboxPlannedTrajectory(bool)));

        FTopCameraState = CameraState::Disabled;
        FBottomCameraState = CameraState::Disabled;
        FMapState = MapState::Disabled;

        // Set a specific callback queue for the camera image topics
        n_private.setCallbackQueue(FWorker->getCallbackQueue());
        // Initialize image transport
        it = new image_transport::ImageTransport(n_private);

        FWorker_thread.start();
    }

    MainWindow::~MainWindow() {
        ros::shutdown();
        FWorker_thread.quit();
        FWorker_thread.wait();
        delete ui;
        delete FManager;
    }

    void MainWindow::RadioButtonTopCameraPressed(int button_id)
    {
        if(button_id == -2 && FTopCameraState != CameraState::Disabled) {
            FTopCameraState = CameraState::Disabled;
            FTopCameraImageSub.shutdown();
            // Remove pixmap
            ui->TopCameraImage->clear();
            // Set black background
            ui->TopCameraImage->setStyleSheet("background-color: black;");
        }
        else {
            if(FTopCameraState != CameraState::Disabled) {
                FTopCameraImageSub.shutdown();
            }
            // Clear label
            ui->TopCameraImage->clear();
            if (button_id == -3 && FTopCameraState != CameraState::RGB) {
                FTopCameraImageSub = it->subscribe(FTopCameraRGBTopicName, 5, &TWorker::cbProcessTopCameraFrame, FWorker);
                FTopCameraState = CameraState::RGB;
            } else if (button_id == -4 && FTopCameraState != CameraState::Depth) {
                FTopCameraImageSub = it->subscribe(FTopCameraDepthTopicName, 5, &TWorker::cbProcessTopCameraFrame, FWorker);
                FTopCameraState = CameraState::Depth;
            }
        }
    }

    void MainWindow::RadioButtonBottomCameraPressed(int button_id)
    {
        if(button_id == -2 && FBottomCameraState != CameraState::Disabled) {
            FBottomCameraState = CameraState::Disabled;
            FBottomCameraImageSub.shutdown();
            // Remove pixmap
            ui->BottomCameraImage->clear();
            // Set black background
            ui->BottomCameraImage->setStyleSheet("background-color: black;");
        }
        else {
            if(FBottomCameraState != CameraState::Disabled) {
                FBottomCameraImageSub.shutdown();
            }
            // Clear label
            ui->BottomCameraImage->clear();
            if (button_id == -3 && FBottomCameraState != CameraState::RGB) {
                FBottomCameraImageSub = it->subscribe(FBottomCameraRGBTopicName, 5, &TWorker::cbProcessBottomCameraFrame, FWorker);
                FBottomCameraState = CameraState::RGB;
            } else if (button_id == -4 && FBottomCameraState != CameraState::Depth) {
                FBottomCameraImageSub = it->subscribe(FBottomCameraDepthTopicName, 5, &TWorker::cbProcessBottomCameraFrame, FWorker);
                FBottomCameraState = CameraState::Depth;
            }
        }
    }

    void MainWindow::RadioButtonMapPressed(int button_id)
    {
        if(button_id == -2 && FMapState != MapState::Disabled) {
            FMapState = MapState::Disabled;
            FDisplayMap->setEnabled(false);
        }
        else if(button_id == -3 && FMapState != MapState::Two_D) {
            FMapState = MapState::Two_D;
        }
        else if(button_id == -4 && FMapState != MapState::Three_D) {
            FMapState = MapState::Three_D;
            FDisplayMap = FManager->createDisplay("rviz/PointCloud2", "3d_map_point_cloud", true);
            FDisplayMap->subProp("Topic")->setValue(FMap3DTopicName.c_str());
            FDisplayMap->subProp("Style")->setValue("Points");
            FDisplayMap->subProp("Size (Pixels)")->setValue(FDisplayPointCloud2SizePixels);
            FDisplayMap->subProp("Decay Time")->setValue(FDisplayPointCloud2DecayTime);
            FDisplayMap->subProp("Color Transformer")->setValue("Intensity");
        }
    }

    void MainWindow::SLOT_UpdateTopCameraImage(const QPixmap& qt_pixmap)
    {
        ui->TopCameraImage->setPixmap(qt_pixmap);
    }

    void MainWindow::SLOT_UpdateBottomCameraImage(const QPixmap& qt_pixmap)
    {
        ui->BottomCameraImage->setPixmap(qt_pixmap);
    }

    void MainWindow::SLOT_CheckboxCurrentPoseTF(bool state) {
        if(state) {
            FDisplayTF = FManager->createDisplay("rviz/TF", "tf_display", true);

        }
        if(!state) {
            FDisplayTF->setEnabled(false);
        }
    }

    void MainWindow::SLOT_CheckboxOdometry(bool state) {
        if(state) {
            FDisplayOdometry = FManager->createDisplay("rviz/Odometry", "robot_odometry", true);
            FDisplayOdometry->subProp("Topic")->setValue(FOdometryTopicName.c_str());
            FDisplayOdometry->subProp("Position Tolerance")->setValue(FOdometryPositionTolerance);
            FDisplayOdometry->subProp("Angle Tolerance")->setValue(FOdometryAngleTolerance);
            FDisplayOdometry->subProp("Keep")->setValue(FOdometrySamplesToKeep);
            FDisplayOdometry->subProp("Shape")->subProp("Alpha")->setValue(FOdometryShapeAlpha);
            FDisplayOdometry->subProp("Shape")->subProp("Shaft Length")->setValue(FOdometryShapeShaftLength);
            FDisplayOdometry->subProp("Shape")->subProp("Head Length")->setValue(FOdometryShapeHeadLength);
        }
        if(!state) {
            FDisplayOdometry->setEnabled(false);
        }
    }

    void MainWindow::SLOT_CheckboxPlannedTrajectory(bool state) {

    }

}

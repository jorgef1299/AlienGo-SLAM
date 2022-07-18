#include <QApplication>
#include "aliengo_gui.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aliengo_gui");

    QApplication gui_app(argc, argv);

    Aliengo::MainWindow* aliengo_window = new Aliengo::MainWindow();
    aliengo_window->show();

    gui_app.exec();

    delete aliengo_window;
}


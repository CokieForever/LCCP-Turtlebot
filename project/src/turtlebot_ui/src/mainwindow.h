#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "detect_marker/detect.hpp"
#include <QMainWindow>
#include "aruco/aruco.h"
#include <QString>

namespace Ui {
class MainWindow;
}

/**
 * @brief The MainWindow class, inherits from QMainWindow and uses the macro Q_OBJECTS in order to call signals and slots
 * Note: The custom messages need to be modified such that they are generated from the packages turtlebot_ui, since
 * there is no individual detect_marker package anymore. Every additional package that is to be included in the user
 * interfaces would get a folder inside turtlebot_ui/src.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public Q_SLOTS:
    void sl_detect_marker_status(QString qsText);
    void sl_run_detect_marker();
    void sl_from_dm_frame(cv::Mat &frame);
Q_SIGNALS:
private:
    Ui::MainWindow *ui; ///< user interface pointer to access elements
    DetectMarker *p_dm; ///< pointer to control the DetectMarker class
    ros::NodeHandle m_nodeHandle; ///< ROS needs a node handler
};

#endif // MAINWINDOW_H

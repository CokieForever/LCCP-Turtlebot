#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "detect_marker/detect.hpp"
#include <QMainWindow>
#include "aruco/aruco.h"
#include <QString>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void hi(QString qsText);

public Q_SLOTS:
    void sl_detect_marker_status(QString qsText);
    void sl_run_detect_marker();
    void sl_from_dm_frame(cv::Mat &frame);

Q_SIGNALS:

private:
    Ui::MainWindow *ui;
    DetectMarker *p_dm;
    ros::NodeHandle m_nodeHandle;
    //cv::Mat &live_frame;
};

#endif // MAINWINDOW_H

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "detectmarker/detectmarker.h"
#include <QMainWindow>
#include "aruco/aruco.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public Q_SLOTS:
    void sl_run_detect_marker();
private:
    Ui::MainWindow *ui;
    DetectMarker *p_dm;
};

#endif // MAINWINDOW_H

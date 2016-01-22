#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    p_dm = new DetectMarker;
    connect(ui->btnMarkerDetection,SIGNAL(clicked()), this, SLOT(sl_run_detect_marker()));
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::sl_run_detect_marker()
{
    //start marker detection here
    dm->Detect();

}

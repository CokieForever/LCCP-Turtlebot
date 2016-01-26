#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTextBrowser>
#include <iostream>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //start marker detection here
     p_dm = new DetectMarker(m_nodeHandle);
     connect(p_dm,SIGNAL(si_status_togui(QString)),this,SLOT(sl_detect_marker_status(QString)));
    connect(p_dm,SIGNAL(si_togui_frame(cv::Mat &)), this, SLOT(sl_from_dm_frame(cv::Mat &)));
    connect(ui->btnMarkerDetection,SIGNAL(clicked()), this, SLOT(sl_run_detect_marker()));

    cout << "cout works.." << endl;

}

void MainWindow::sl_from_dm_frame(cv::Mat &frame)
{
  //cv::imshow("Marker Detection From GUI", frame);
  //display frame in widget
  cv::resize(frame, frame,cv::Size(ui->labelFrame->width(),ui->labelFrame->height()));
  cvtColor(frame,frame,CV_BGR2RGB);
  QImage img = QImage((uchar*) frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
  QPixmap pix = QPixmap::fromImage(img);
  ui->labelFrame->setPixmap(pix);
}

void MainWindow::sl_detect_marker_status(QString qsText)
{

  ui->txtbStatus->append(qsText);

}

void MainWindow::hi(QString qsText)
{
  ui->txtbStatus->append(qsText);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::sl_run_detect_marker()
{

         cout << "Starting detection..." << endl;
         p_dm->Start();
         p_dm->Detect();

}

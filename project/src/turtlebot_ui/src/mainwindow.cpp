#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTextBrowser>
#include <iostream>
/**
 * @brief connect various signals with slots
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
     p_dm = new DetectMarker(m_nodeHandle);
     connect(p_dm,SIGNAL(si_status_togui(QString)),this,SLOT(sl_detect_marker_status(QString)));
    connect(p_dm,SIGNAL(si_togui_frame(cv::Mat &)), this, SLOT(sl_from_dm_frame(cv::Mat &)));
    connect(ui->btnMarkerDetection,SIGNAL(clicked()), this, SLOT(sl_run_detect_marker()));
}
/**
 * @brief Slot to transfer the frames from the marker detection to the user interface
 * @param frame cv::Mat
 */
void MainWindow::sl_from_dm_frame(cv::Mat &frame)
{
  cv::resize(frame, frame,cv::Size(ui->labelFrame->width(),ui->labelFrame->height()));
  cvtColor(frame,frame,CV_BGR2RGB);
  QImage img = QImage((uchar*) frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
  QPixmap pix = QPixmap::fromImage(img);
  ui->labelFrame->setPixmap(pix);
}
/**
 * @brief Slot called when there is a status signaled at detect_marker
 * @param qsText QString
 */
void MainWindow::sl_detect_marker_status(QString qsText)
{
  ui->txtbStatus->append(qsText);
}
/**
 * @brief Delete instance of user interface
 */
MainWindow::~MainWindow()
{
    delete ui;
}
/**
 * @brief Slot that is called when the button is clicked to start the marker detection
 */
void MainWindow::sl_run_detect_marker()
{
         ui->btnMarkerDetection->setEnabled(false);
         ui->btnPlaySuperMario->setEnabled(true);
         ui->btnSearchMarkers->setEnabled(true);
         p_dm->Start();
         p_dm->Detect();
}

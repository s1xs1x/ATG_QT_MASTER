#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "atg_image.h"
#include <opencv2/opencv.hpp>
#include "flash.h"
uchar mt9v03x_image[MT9V03X_H][MT9V03X_W];
uchar ostu_img[MT9V03X_H][MT9V03X_W];
uchar adaptive_img[MT9V03X_H][MT9V03X_W];
image_t img_raw = DEF_IMAGE(NULL, MT9V03X_W, MT9V03X_H);

uint16 Ostu_Thres;/**< 大津法后的阈值*/

void Qt_img_show();
void Qt_draw_line();
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::mainWindow)
{
    ui->setupUi(this);
        //cv::Mat image = cv::imread("E:/qImage/new1.BMP",cv::IMREAD_GRAYSCALE);
cv::Mat image = cv::imread("E:/Upper computer/ATG_Master/2.BMP",cv::IMREAD_GRAYSCALE);
    // create image window named "My Image"
//    cv::namedWindow("My Image");
//    // show the image on window
//    cv::imshow("My Image", image);



    for (int i = 0; i < MT9V03X_H; ++i)
    {
        for (int j = 0; j < MT9V03X_W; ++j)
        {
            mt9v03x_image[i][j] = image.at<uchar>(i, j);
        }
    }

    //大津法
    Ostu_Thres=otsuThreshold(&mt9v03x_image[0][0], 188, 120);
    thres_hold_255(mt9v03x_image[0],ostu_img[0],188,120,Ostu_Thres);
    QImage Q_image_ostu(ostu_img[0], 188, 120, QImage::Format_Grayscale8);
    ui->label_img_ostu->setPixmap(QPixmap::fromImage(Q_image_ostu));
    ui->label_img_ostu->show();

    //自适应二值化

    adaptiveThreshold(mt9v03x_image[0],adaptive_img[0],MT9V03X_W, MT9V03X_H, adaptive_Block,clip_value);
    QImage Q_image_adaptive(adaptive_img[0], 188, 120, QImage::Format_Grayscale8);
    ui->label_img_adp->setPixmap(QPixmap::fromImage(Q_image_adaptive));
    ui->label_img_adp->show();


    img_raw.data = mt9v03x_image[0];
    process_image();

    //创建一个灰度图
    QImage Q_image(mt9v03x_image[0], 188, 120, QImage::Format_Grayscale8);
    //转成彩色图以便画点 左红右蓝
    QImage Q_image_666;
    Q_image_666=Q_image.convertToFormat(QImage::QImage::Format_RGB666);

    QColor RED_RGB666(255,0,0);
    QColor BLUE_RGB666(0,0,255);

    //显示左右边线
    if(ipts0_num>0)
        for (int i=0;i<ipts0_num;i++)
        {
            Q_image_666.setPixel(ipts0[i][0],ipts0[i][1],RED_RGB666.rgb());
        }

    if(ipts1_num>0)
        for (int i=0;i<ipts1_num;i++)
        {
            Q_image_666.setPixel(ipts1[i][0],ipts1[i][1],BLUE_RGB666.rgb());
        }


    qDebug()<<ipts0_num;


//    ui->label_img->setPixmap(QPixmap::fromImage(Q_image_666));
//    ui->label_img->show();


    ui->plainTextEdit_0->insertPlainText("ipts0_num:"+QString::number(ipts0_num));
    ui->plainTextEdit_0->appendPlainText("ipts1_num:"+QString::number(ipts1_num));

    ui->plainTextEdit_0->appendPlainText("rpts0_num:"+QString::number(rpts0_num));
    ui->plainTextEdit_0->appendPlainText("rpts1_num:"+QString::number(rpts1_num));


    QPixmap scaledPixmap = QPixmap::fromImage(Q_image_666).scaled(ui->label_img->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

    // Set the QPixmap to QLabel
    ui->label_img->setPixmap(scaledPixmap);

    // Set the QLabel to resize along with the QPixmap
    ui->label_img->setScaledContents(true);

}

MainWindow::~MainWindow()
{
    delete ui;
}


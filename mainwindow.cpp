#include "mainwindow.h"
#include "ui_mainwindow.h"

geometry_msgs::Twist JOYBUF;
Ui::MainWindow*      MYUI;

float LINEAR_VEL;//su du
float ANGULAR_VEL;//jiao su du
int   GEAR;
int   MODEL;
QPushButton *PMODEL[4];






MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //kai shi biao zhi wei
    begin=false;
    //qi dong UI
    ui->setupUi(this);
    MYUI=ui;
    PMODEL[0]=ui->Mode_1;
    PMODEL[1]=ui->Mode_2;
    PMODEL[2]=ui->Mode_3;
    PMODEL[3]=ui->Mode_4;
    MODEL=3;
    ui->brake->setFlat(true);
    PMODEL[MODEL]->setStyleSheet("background-color:green;color:white");
    //qi dong ji shi qi
    QTimer *timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timerUpDate()));
    timer->start(100);

    //chu shi hua cha jian
    initDisplayWidgets();


    STREAM.precision(12);



}

MainWindow::~MainWindow()
{

    myProcess->kill();
    system("../../../../src/ui_package/stop.sh ");
    delete ui;
}

void MainWindow::initDisplayWidgets()
{
    ui->progressBar->setRange(0,50000-1);
    ui->progressBar->setValue(49999);
    ui->sliderLinearVel->setRange(0,100-1);
    ui->sliderLinearVel->setValue(0);
    ui->sliderAngularVel->setRange(0,100-1);
    ui->sliderAngularVel->setValue(50);
}
void MainWindow::on_pushButton_clicked()
{
    if(begin==false)
    {

        //qi dong shou bing jiao ben
        myProcess  =  new QProcess ( this ) ;
        myProcess->start("../../../../src/ui_package/run.sh");
        //myProcess->waitForFinished(2000);
        //QByteArray qout = myProcess->readAllStandardOutput();
        //QString qstr = qout;
        //std::string str = qstr.toStdString();
        //std::cerr<<str<<std::endl;

        //fa bu yu ding yue
        subJoy  = nh.subscribe("joy",1000,&JoyMessageReceived);
        pubVel  = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1000);

        //lian jie biao zhi
        ui->label->setStyleSheet("background-color:green;color:white");
        ui->label->setText("Already connect");


        //chu shi hua su du

        LINEAR_VEL=0;//su du
        ANGULAR_VEL=0;//jiao su du
        GEAR=0;

        begin= true ;

    }else
    {
         myProcess->kill();
         delete myProcess;
         ui->label->setText("Please connect to master.");
         ui->label->setStyleSheet("background-color:red;color:white");
         system(" ../../../../src/ui_package/stop.sh ");
         begin=false;
    }
}
void MainWindow::on_RqtButton_clicked()
{
    QProcess  *Process=new QProcess ( this ) ;
    Process->start(" rqt_graph");
}
void MainWindow::timerUpDate()
{
    if(begin)
    {
        ros::spinOnce();
        pubVel.publish(JOYBUF);
    }

    //turtlesim::Pose d;
    //d.x=0;
    //d.y=0;
    //d.theta=0;
    //PoseMessageReceived(d);
}

//ding yue shou bing xin xi
void JoyMessageReceived(const sensor_msgs::Joy &msg)
{
    JOYBUF.linear.x=double(msg.axes[1]/5.0)*double(GEAR);
    JOYBUF.angular.z=double(msg.axes[3]);
    if(msg.axes[1]>0.5)
    {
         MYUI->pushButton_UP->setFlat(false);
         MYUI->pushButton_UP->setStyleSheet("background-color:green;color:white");
    }else if(msg.axes[1]<-0.5)
    {
        MYUI->pushButton_DOWN->setFlat(false);
        MYUI->pushButton_DOWN->setStyleSheet("background-color:green;color:white");
    }else
    {
        MYUI->pushButton_UP->setFlat(true);
        MYUI->pushButton_DOWN->setFlat(true);
        MYUI->pushButton_UP->setStyleSheet("background-color:red;color:black");
        MYUI->pushButton_DOWN->setStyleSheet("background-color:red;color:black");
    }
    if(msg.axes[3]>0.5)
    {
         MYUI->pushButton_LEFT->setFlat(false);
         MYUI->pushButton_LEFT->setStyleSheet("background-color:green;color:white");
    }else if(msg.axes[3]<-0.5)
    {
        MYUI->pushButton_RIGHT->setFlat(false);
        MYUI->pushButton_RIGHT->setStyleSheet("background-color:green;color:white");
    }else
    {
        MYUI->pushButton_LEFT->setFlat(true);
        MYUI->pushButton_RIGHT->setFlat(true);
        MYUI->pushButton_LEFT->setStyleSheet("background-color:red;color:black");
        MYUI->pushButton_RIGHT->setStyleSheet("background-color:red;color:black");
    }
    if(msg.buttons[4]>0.5)
    {
         MYUI->GearRise->setStyleSheet("background-color:green;color:white");
         GEAR>=5?GEAR=5:GEAR++;
         MYUI->GearNumber->display(GEAR);
    }else
    {
         MYUI->GearRise->setStyleSheet("color:black");
    }
    if(msg.buttons[5]>0.5)
    {
         MYUI->GearDrop->setStyleSheet("background-color:green;color:white");
         if(GEAR>0)
             GEAR--;
         MYUI->GearNumber->display(GEAR);
    }else
    {
         MYUI->GearDrop->setStyleSheet("color:black");
    }
    if(msg.axes[5]<0.5||msg.axes[2]<0.5)
    {
         JOYBUF.linear.x=0;
         JOYBUF.angular.z=0;
         MYUI->brake->setFlat(false);
         MYUI->brake->setStyleSheet("background-color:red;color:white");

    }else
    {
         MYUI->brake->setFlat(true);
         MYUI->brake->setStyleSheet("color:black");
    }

    if(msg.axes[7]>0.5)
        MODEL=0;
    else if(msg.axes[7]<-0.5)
        MODEL=2;
    else if(msg.axes[6]<-0.5)
        MODEL=1;
    else if(msg.axes[6]>0.5)
        MODEL=3;
    for(int i=0;i<4;i++)
        PMODEL[i]->setStyleSheet("color:black");
    PMODEL[MODEL]->setStyleSheet("background-color:green;color:white");
    MYUI->sliderLinearVel->setValue(int(JOYBUF.linear.x*100));
    MYUI->sliderAngularVel->setValue(int(50+JOYBUF.angular.z*50));

}


void PoseMessageReceived(const turtlesim::Pose &msg)
{
    /*int a=int(msg.x);
    int b=int(msg.y);
    int c=int(msg.theta);

    cv::Point mycenter= cv::Point(marke.cols/2,marke.rows/2);
    double angle= -90+int(double(c)/625.0*360.0);
    double scale= 0.5;
    cv::Mat rotMat(2,3,CV_32FC1);//(2,3,cv::CV_32FC1);
    rotMat=cv::getRotationMatrix2D(mycenter,angle,scale);
    cv::Mat markedstImage_rotatr;
    cv::warpAffine(marke,markedstImage_rotatr,rotMat,marke.size());


    cv::Mat maskdstImage_rotatr;
    cv::warpAffine(mask,maskdstImage_rotatr,rotMat,mask.size());


    //cv::imshow("s",dstImage_rotatr);
    cv::Mat i;
    cv::Mat imageROI;
    i=map.clone();
    imageROI=i(cv::Rect(a/1,b/1,markedstImage_rotatr.cols,markedstImage_rotatr.rows));

    markedstImage_rotatr.copyTo(imageROI,maskdstImage_rotatr);
    //cv::addWeighted(imageROI,0.5,marke,0.5,0.0,imageROI);
    cv::imshow("Map",i);
*/
}


void MainWindow::on_pushButton_2_clicked()
{
    QString qs;
    qs = ui->lineEdit_2->text();
    std::string str = qs.toStdString();
    float gps[6];

    STREAM.clear();
    STREAM<<str;
    STREAM>>gps[0];
    STREAM>>gps[1];
    STREAM>>gps[2];
    STREAM>>gps[3];
    STREAM>>gps[4];
    STREAM>>gps[5];

    CMap.SetMapPose(gps[0], gps[1], gps[2], gps[3], gps[4], gps[5]);
    CMap.ShowMap();
    cvMoveWindow("Map",0,0);

    //STREAM.clear();
    //STREAM<<gps[0];
    //STREAM>>str;
    //qs = QString::fromStdString(str);
    //ui->lineEdit->setText(qs);
}

void MainWindow::on_pushButton_3_clicked()
{
    QString qs;
    qs = ui->lineEdit->text();
    std::string str = qs.toStdString();
    float num,gps_x,gps_y;

    STREAM.clear();
    STREAM<<str;
    STREAM>>num;
    STREAM>>gps_x;
    STREAM>>gps_y;

    CMap.SetDoor(num,gps_x,gps_y);
    CMap.ShowMap();

    std::string tmp;
    STREAM.clear();
    STREAM<<(num+1);
    STREAM<<' ';
    STREAM<<(gps_x+5);
    STREAM<<' ';
    STREAM<<gps_y;
    STREAM>>str;
    STREAM>>tmp;
    str +=(' '+tmp);
    STREAM>>tmp;
    str +=(' '+tmp);
    qs = QString::fromStdString(str);
    ui->lineEdit->setText(qs);

}

void MainWindow::on_pushButton_4_clicked()
{
    QString qs;
    qs = ui->lineEdit_3->text();
    std::string str = qs.toStdString();
    float num;

    STREAM.clear();
    STREAM<<str;
    STREAM>>num;

    CMap.DeleteDoor(num);
    CMap.ShowMap();
}

void MainWindow::on_pushButton_5_clicked()
{
    QString qs;
    qs = ui->lineEdit_4->text();
    std::string str = qs.toStdString();
    float num;

    STREAM.clear();
    STREAM<<str;
    STREAM>>num;

    CMap.selectDoor=int(num);
    CMap.ShowMap();
}

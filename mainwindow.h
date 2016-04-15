#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QApplication>
#include <QtCore>
#include <QKeyEvent>
#include <QEvent>
#include <QProcess>
#include <QPushButton>

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include "mapcv.h"





namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();


private slots:
    void on_pushButton_clicked();
    void on_RqtButton_clicked();
    void timerUpDate();



    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

private:

    //ding ti UI
    Ui::MainWindow *ui;

    //ding yi ROS jie dian
    ros::NodeHandle nh;

    //ding yue shou bing xin xi de bian liang
    ros::Subscriber subJoy;

    //ding yue wei zhi de ding yi bian liang
    ros::Subscriber subPose;

    //ding yue su du de ding yi bian liang
    ros::Subscriber subVel;

    //fa bu su du de ding yi bian liang
    ros::Publisher  pubVel;

    bool begin;
    QProcess  *myProcess;

    std::stringstream STREAM;
    MapCV CMap;
private:
    void initDisplayWidgets();

};

void JoyMessageReceived(const sensor_msgs::Joy &msg);
void PoseMessageReceived(const turtlesim::Pose &msg);



#endif // MAINWINDOW_H

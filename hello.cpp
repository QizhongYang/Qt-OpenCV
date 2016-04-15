
#include "mainwindow.h"

int main(int argc,char** argv )
{

    ros::init(argc,argv,"hello");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}

#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("VSLS");

    MainWindow window;

    //window.setStyleSheet("background:rgb(0,0,0)");
    //window.showFullScreen();
    window.showNormal();
    return app.exec();
}
